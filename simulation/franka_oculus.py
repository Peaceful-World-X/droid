#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Oculus手柄实时控制Franka机械臂
Date: 2025-10-20
Author: xwy
"""

import json
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from common import setup_logger
from franka_r7arm import FrankaR7arm
from scipy.spatial.transform import Rotation as R

droid_path = Path(__file__).parent.parent / "droid"
sys.path.insert(0, str(droid_path / "oculus_reader" / "oculus_reader"))
from reader import OculusReader


class FrankaOculusController:
    """Franka机械臂的Oculus手柄控制器"""
    def __init__(self, control_frequency: float = 30.0, enable_recording: bool = True,  log_output: str = "both"):
        """ 初始化Franka Oculus控制器
        Args:
            control_frequency: 控制频率 (Hz)
            enable_recording: 是否启用数据记录
            log_output: 日志输出位置
        """
        # 控制参数
        self.control_frequency = control_frequency
        self.enable_recording = enable_recording
        self.logger = setup_logger("Franka_Oculus", output=log_output)

        # 初始化机器人
        self.logger.info("🤖 初始化机械臂...")
        self.robot = FrankaR7arm()
        if not self.robot.connect():
            raise RuntimeError("❌ 机械臂连接失败")

        # 初始化Oculus读取器
        self.logger.info("🎮 初始化Oculus手柄...")
        self.oculus_reader = OculusReader()
        time.sleep(0.5)

        # 状态变量
        self.running = False
        self.paused = False
        self.control_thread = None
        self.oculus_thread = None

        # Oculus数据缓存（线程安全）
        self.latest_oculus_data = {}
        self.oculus_data_lock = threading.Lock()

        # 按钮状态记录（用于防抖）
        self.button_A_prev = False
        self.button_B_prev = False

        # 数据记录
        self.trajectory_data = deque()
        self.logger.info("✅ 初始化完成")

    def start(self):
        """启动控制循环"""
        if self.running:
            self.logger.warning("⚠️ 控制器已在运行")
            return

        self.running = True
        self.paused = False

        # 启动Oculus数据采集线程（高优先级）
        self.oculus_thread = threading.Thread(target=self._oculus_reader_loop, daemon=True)
        self.oculus_thread.start()

        # 等待Oculus线程启动
        time.sleep(0.1)

        # 启动控制线程（普通优先级）
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        self.logger.info("🚀 控制器已启动")

    def pause(self):
        """暂停控制"""
        self.paused = True
        self.logger.info("⏸️ 控制已暂停")

    def resume(self):
        """恢复控制"""
        self.paused = False
        self.logger.info("▶️ 控制已恢复")

    def stop(self):
        """停止控制"""
        self.running = False

        # 等待线程结束
        if self.oculus_thread:
            self.oculus_thread.join(timeout=1.0)
        if self.control_thread:
            self.control_thread.join(timeout=2.0)

        self.robot.stop_execution()
        self.logger.info("⏹️ 控制器已停止")

    def save(self):
        """保存记录的轨迹数据"""
        if not self.enable_recording or len(self.trajectory_data) == 0:
            self.logger.warning("⚠️ 没有数据可保存")
            return

        # 生成文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}.json"

        # 保存数据
        data_dict = {
            'metadata': {
                'control_frequency': self.control_frequency,
                'timestamp': timestamp,
                'num_samples': len(self.trajectory_data)
            },
            'trajectory': list(self.trajectory_data)
        }

        with open(filename, 'w') as f:
            json.dump(data_dict, f, indent=2)

        self.logger.info(f"💾 轨迹已保存: {filename} ({len(self.trajectory_data)} 个数据点)")

    def cleanup(self):
        """清理资源"""
        self.logger.info("🧹 清理资源...")
        self.stop()

        # 断开机器人连接
        if self.robot:
            self.robot.disconnect()

        # 停止Oculus读取器
        if self.oculus_reader:
            self.oculus_reader.stop()

        self.logger.info("✅ 清理完成")

    # =======================================
    def get_observation(self) -> Optional[Dict]:
        """获取机器人当前观测 """
        status = self.robot.get_current_status()
        if not status:
            return None

        ee_pose = status['ee_pose']
        position = np.array(ee_pose.translation)
        quaternion = np.array(ee_pose.quaternion)
        rotation = R.from_quat(quaternion)
        euler = rotation.as_euler('xyz', degrees=False)

        return {
            'pose_6d': np.concatenate([position, euler]).tolist(),
            'joint_positions': status['joint_positions'],
            'gripper_position': status['gripper_position']
        }

    def get_oculus_data(self, oculus_output: Tuple) -> Dict:
        """
        解析Oculus手柄的输出数据
        Args:
            oculus_output: oculus_reader.get_transformations_and_buttons() 如下
            ({'r': array([[-0.544345 ,  0.8292387,  0.838352 ,  0.0633542],
                        [ 0.344583 , -0.903392 ,  0.255237 , -0.497279 ],
                        [ 0.764821 ,  0.427819 ,  0.481685 , -0.14618 ],
                        [ 0.     ,  0.     ,  0.     ,  1.     ]])},
             {'A': False, 'B': False, 'RTHU': True, 'RJ': False, 'RG': False, 'RTr': False, 'rightJS': (0.0, 0.0), 'rightTrig': (0.0, 0.0), 'rightGrip': (0.0,)})
        Returns:
            包含以下字段的字典:
            - pose_6d: [x, y, z, roll, pitch, yaw]
            - button_A: bool, A按钮状态
            - button_B: bool, B按钮状态
            - grip: float, 右手握力值 (0.0-1.0)
        """
        transforms, buttons = oculus_output

        # 提取右手变换矩阵
        if 'r' in transforms:
            transform_matrix = transforms['r']

            # 提取末端位姿
            position = transform_matrix[:3, 3]  # 从变换矩阵提取位置
            rotation_matrix = transform_matrix[:3, :3]  # 从变换矩阵提取旋转矩阵
            rotation = R.from_matrix(rotation_matrix)   # 转换为Rotation对象
            euler = rotation.as_euler('xyz', degrees=False) # 获取欧拉角 [roll, pitch, yaw] 弧度制
            pose_6d = np.concatenate([position, euler])

            # 提取按钮状态
            button_A = buttons.get('A', False)
            button_B = buttons.get('B', False)

            # 提取右手握力值
            right_grip_tuple = buttons.get('rightGrip', (0.0,))
            right_grip = float(right_grip_tuple[0]) if isinstance(right_grip_tuple, tuple) else float(right_grip_tuple)

            return {
                'pose_6d': pose_6d.tolist(),
                'button_A': button_A,
                'button_B': button_B,
                'grip': right_grip,
            }
        return {}

    def _oculus_reader_loop(self):
        """Oculus数据采集线程（高优先级）- 独立运行，专门负责读取Oculus数据"""

        self.logger.info("🎮 Oculus数据采集线程已启动")
        read_rate = 1.0 / 50.0
        while self.running:
            start_time = time.time()

            try:
                oculus_output = self.oculus_reader.get_transformations_and_buttons()
                oculus_data = self.get_oculus_data(oculus_output)
                if oculus_data:
                    with self.oculus_data_lock:
                        self.latest_oculus_data = oculus_data
                    self._handle_buttons(oculus_data)
            except Exception as e:
                self.logger.error(f"❌ Oculus读取错误: {e}")

            elapsed = time.time() - start_time
            if elapsed < read_rate:
                time.sleep(read_rate - elapsed)

        self.logger.info("🎮 Oculus数据采集线程已停止")

    def _control_loop(self):
        """控制循环主函数 - 使用缓存的Oculus数据控制机械臂"""
        loop_rate = 1.0 / self.control_frequency
        self.logger.info(f"🤖 机械臂控制循环已启动 (频率: {self.control_frequency} Hz)")

        while self.running:
            start_time = time.time()
            if self.paused:
                time.sleep(loop_rate)
                continue

            try:
                with self.oculus_data_lock:
                    oculus_data = self.latest_oculus_data.copy()
                if not oculus_data:
                    time.sleep(loop_rate)
                    continue

                # 控制机械臂
                pose_6d = oculus_data['pose_6d']
                if not self.robot.send_cartesian_pose(pose_6d):
                    self.logger.warning("⚠️ 机械臂控制报错")

                # 控制夹爪
                gripper_pos = oculus_data['grip']
                if not self.robot.send_gripper_position(gripper_pos):
                    self.logger.warning("⚠️ 夹爪控制报错")

                # 记录数据
                if self.enable_recording:
                    self._record_data(oculus_data)

            except Exception as e:
                self.logger.error(f"❌ 控制循环错误: {e}")
                import traceback
                traceback.print_exc()

            elapsed = time.time() - start_time
            if elapsed < loop_rate:
                time.sleep(loop_rate - elapsed)

        self.logger.info("🤖 机械臂控制循环已停止")

    def _record_data(self, oculus_data: Dict):
        """记录轨迹数据"""
        robot_obs = self.get_observation()
        if robot_obs:
            data_point = {
                'timestamp': time.time(),
                # 'oculus': oculus_data,
                'robot': robot_obs
            }
            self.trajectory_data.append(data_point)

    def _handle_buttons(self, oculus_data: Dict):
        """处理按钮事件（带防抖）"""
        button_A = oculus_data.get('button_A', False)
        button_B = oculus_data.get('button_B', False)

        # A按钮：保存数据
        if button_A and not self.button_A_prev:
            self.save()

        # B按钮：暂停/恢复
        if button_B and not self.button_B_prev:
            if self.paused:
                self.resume()
            else:
                self.pause()

        # 更新按钮状态
        self.button_A_prev = button_A
        self.button_B_prev = button_B

# ==================== 演示函数 ====================
def main():
    """主函数"""

    controller = None
    try:
        controller = FrankaOculusController()
        controller.start()
        while controller.running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n⚠️ 检测到键盘中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
    finally:
        if controller:
            controller.cleanup()


if __name__ == "__main__":
    main()
