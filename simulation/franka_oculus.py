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
import yaml
from common import setup_logger
from franka_r7arm import FrankaR7arm
from scipy.spatial.transform import Rotation as R

droid_path = Path(__file__).parent.parent / "droid"
from oculus_reader.reader import OculusReader


class FrankaOculusController:
    """Franka机械臂的Oculus手柄控制器"""
    def __init__(self, config_path: str = "config.yaml", log_output: str = "both"):
        """ 初始化Franka Oculus控制器
        Args:
            config_path: 配置文件路径
            log_output: 日志输出位置
        """
        # 加载配置文件
        self.config = self._load_config(config_path)

        # 控制参数
        self.control_frequency = self.config['oculus_control']['control_frequency']
        self.enable_recording = self.config['oculus_control']['enable_recording']
        self.data_save_path = self.config['oculus_control']['data_save_path']
        self.position_scale = self.config['oculus_control']['position_scale']
        self.rotation_scale = self.config['oculus_control']['rotation_scale']
        self.max_position_delta = self.config['oculus_control']['max_position_delta']
        self.max_rotation_delta = self.config['oculus_control']['max_rotation_delta']

        self.logger = setup_logger("Franka_Oculus", output=log_output)

        # 确保数据保存目录存在
        os.makedirs(self.data_save_path, exist_ok=True)

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
        self.control_thread = None
        self.oculus_thread = None

        # Oculus数据缓存（线程安全）
        self.latest_oculus_data = {}
        self.oculus_data_lock = threading.Lock()

        # 按钮状态记录（用于防抖）
        self.button_A_prev = False
        self.button_B_prev = False

        # 相对位姿控制相关变量
        self.oculus_initial_pose = None  # Oculus手柄初始位姿 [position(3), quaternion(4)]
        self.robot_initial_pose = None   # 机械臂初始位姿 [position(3), euler(3)]
        self.control_active = False      # 控制是否激活
        self.control_paused = False      # 控制是否暂停
        self.control_state_lock = threading.Lock()  # 控制状态锁

        # 数据记录（线程安全）
        self.trajectory_data = deque()
        self.trajectory_data_lock = threading.Lock()

        self.logger.info("✅ 初始化完成")
        self.logger.info(f"   位置缩放: {self.position_scale}, 旋转缩放: {self.rotation_scale}")
        self.logger.info(f"   数据保存路径: {self.data_save_path}")

    def _load_config(self, config_path: str) -> Dict:
        """加载配置文件"""
        config_file = Path(__file__).parent / config_path
        if not config_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_file}")

        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        return config

    def start(self):
        """启动控制循环"""
        if self.running:
            self.logger.warning("⚠️ 控制器已在运行")
            return

        self.running = True

        # 启动Oculus数据采集线程（高优先级）
        self.oculus_thread = threading.Thread(target=self._oculus_reader_loop, daemon=True)
        self.oculus_thread.start()

        # 等待Oculus线程启动
        time.sleep(0.1)

        # 启动控制线程（普通优先级）
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        self.logger.info("🚀 控制器已启动")

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
        """保存记录的轨迹数据（线程安全）"""
        with self.trajectory_data_lock:
            if not self.enable_recording or len(self.trajectory_data) == 0:
                self.logger.warning("⚠️ 没有数据可保存")
                return

            # 生成文件名（包含毫秒避免冲突）
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            milliseconds = int((time.time() % 1) * 1000)
            filename = f"trajectory_{timestamp}_{milliseconds:03d}.json"
            filepath = os.path.join(self.data_save_path, filename)

            # 保存数据（复制一份避免在保存过程中被修改）
            data_dict = {
                'metadata': {
                    'control_frequency': self.control_frequency,
                    'timestamp': timestamp,
                    'num_samples': len(self.trajectory_data),
                    'position_scale': self.position_scale,
                    'rotation_scale': self.rotation_scale,
                },
                'trajectory': list(self.trajectory_data)
            }

        # 在锁外进行文件写入
        try:
            with open(filepath, 'w') as f:
                json.dump(data_dict, f, indent=2)
            self.logger.info(f"💾 轨迹已保存: {filepath} ({data_dict['metadata']['num_samples']} 个数据点)")
        except Exception as e:
            self.logger.error(f"❌ 保存文件失败: {e}")

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

    def get_status(self) -> Dict:
        """获取当前控制状态（线程安全）"""
        with self.control_state_lock:
            control_active = self.control_active
            control_paused = self.control_paused

        with self.trajectory_data_lock:
            data_count = len(self.trajectory_data)

        return {
            'running': self.running,
            'control_active': control_active,
            'control_paused': control_paused,
            'recording_enabled': self.enable_recording,
            'data_count': data_count,
        }

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
             {'A': False, 'B': False, 'RTHU': True, 'RJ': False, 'RG': False, 'RTr': False, 'rightJS': (0.0, 0.0), 'rightTrig': (0.0,), 'rightGrip': (0.0,)})
        Returns:
            包含以下字段的字典:
            - position: [x, y, z]
            - quaternion: [x, y, z, w] (四元数)
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
            quaternion = rotation.as_quat()  # 获取四元数 [x, y, z, w]

            # 提取按钮状态
            button_A = buttons.get('A', False)
            button_B = buttons.get('B', False)

            # 提取右手握力值
            right_grip_tuple = buttons.get('rightGrip', (0.0,))
            right_grip = float(right_grip_tuple[0]) if isinstance(right_grip_tuple, tuple) else float(right_grip_tuple)

            return {
                'position': position.tolist(),
                'quaternion': quaternion.tolist(),
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

    def _calculate_relative_pose(self, current_oculus_data: Dict) -> Optional[np.ndarray]:
        """
        计算相对位姿（带缩放和限位）
        Args:
            current_oculus_data: 当前Oculus数据
        Returns:
            相对位姿 [delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw]
        """
        if self.oculus_initial_pose is None:
            return None

        # 当前Oculus位姿
        curr_pos = np.array(current_oculus_data['position'])
        curr_quat = np.array(current_oculus_data['quaternion'])

        # 初始Oculus位姿
        init_pos = self.oculus_initial_pose[:3]
        init_quat = self.oculus_initial_pose[3:]

        # 计算位置差并应用缩放
        delta_pos = (curr_pos - init_pos) * self.position_scale

        # 限制位置变化
        delta_pos_norm = np.linalg.norm(delta_pos)
        if delta_pos_norm > self.max_position_delta:
            delta_pos = delta_pos / delta_pos_norm * self.max_position_delta

        # 计算旋转差（相对旋转）
        # R_relative = R_current * R_initial^(-1)
        R_init = R.from_quat(init_quat)
        R_curr = R.from_quat(curr_quat)
        R_relative = R_curr * R_init.inv()

        # 转换为欧拉角并应用缩放
        delta_euler = R_relative.as_euler('xyz', degrees=False) * self.rotation_scale

        # 限制旋转变化
        delta_euler_norm = np.linalg.norm(delta_euler)
        if delta_euler_norm > self.max_rotation_delta:
            delta_euler = delta_euler / delta_euler_norm * self.max_rotation_delta

        # 返回相对位姿
        relative_pose = np.concatenate([delta_pos, delta_euler])
        return relative_pose

    def _apply_relative_to_absolute(self, relative_pose: np.ndarray) -> np.ndarray:
        """
        将相对位姿应用到机械臂初始位姿上，得到绝对控制指令
        Args:
            relative_pose: 相对位姿 [delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw]
        Returns:
            绝对位姿 [x, y, z, roll, pitch, yaw]
        """
        if self.robot_initial_pose is None:
            return None

        # 分离相对位姿的位置和旋转
        delta_pos = relative_pose[:3]
        delta_euler = relative_pose[3:]

        # 机械臂初始位姿
        init_pos = np.array(self.robot_initial_pose[:3])
        init_euler = np.array(self.robot_initial_pose[3:])

        # 计算绝对位置
        abs_pos = init_pos + delta_pos

        # 计算绝对旋转
        # R_absolute = R_relative * R_initial
        R_init = R.from_euler('xyz', init_euler, degrees=False)
        R_delta = R.from_euler('xyz', delta_euler, degrees=False)
        R_abs = R_delta * R_init
        abs_euler = R_abs.as_euler('xyz', degrees=False)

        # 返回绝对位姿
        absolute_pose = np.concatenate([abs_pos, abs_euler])
        return absolute_pose

    def _control_loop(self):
        """控制循环主函数 - 使用相对位姿控制机械臂"""
        loop_rate = 1.0 / self.control_frequency
        self.logger.info(f"🤖 机械臂控制循环已启动 (频率: {self.control_frequency} Hz)")

        while self.running:
            start_time = time.time()

            # 线程安全地检查控制状态
            with self.control_state_lock:
                should_control = self.control_active and not self.control_paused

            # 如果未激活控制或已暂停，等待
            if not should_control:
                time.sleep(loop_rate)
                continue

            try:
                with self.oculus_data_lock:
                    oculus_data = self.latest_oculus_data.copy()
                if not oculus_data:
                    time.sleep(loop_rate)
                    continue

                # 计算相对位姿
                relative_pose = self._calculate_relative_pose(oculus_data)
                if relative_pose is None:
                    time.sleep(loop_rate)
                    continue

                # 转换为绝对位姿控制指令
                absolute_pose = self._apply_relative_to_absolute(relative_pose)
                if absolute_pose is None:
                    time.sleep(loop_rate)
                    continue

                # 控制机械臂
                if not self.robot.send_cartesian_pose(absolute_pose.tolist()):
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
        """记录轨迹数据（线程安全）"""
        robot_obs = self.get_observation()
        if robot_obs:
            data_point = {
                'timestamp': time.time(),
                # 'oculus': oculus_data,
                'robot': robot_obs
            }
            with self.trajectory_data_lock:
                self.trajectory_data.append(data_point)

    def _handle_buttons(self, oculus_data: Dict):
        """处理按钮事件（带防抖）"""
        button_A = oculus_data.get('button_A', False)
        button_B = oculus_data.get('button_B', False)

        # A按钮：开始/暂停控制（可恢复）
        if button_A and not self.button_A_prev:
            if not self.control_active:
                # 首次启动：记录初始位姿并开始控制
                self._start_control(oculus_data)
            elif self.control_paused:
                # 恢复控制
                self._resume_control()
            else:
                # 暂停控制
                self._pause_control()

        # B按钮：保存数据并彻底结束控制
        if button_B and not self.button_B_prev:
            self._save_and_stop()

        # 更新按钮状态
        self.button_A_prev = button_A
        self.button_B_prev = button_B

    def _start_control(self, oculus_data: Dict):
        """开始控制：记录Oculus和机械臂的初始位姿（线程安全）"""
        # 记录Oculus初始位姿
        oculus_pos = np.array(oculus_data['position'])
        oculus_quat = np.array(oculus_data['quaternion'])

        # 获取并记录机械臂初始位姿
        robot_obs = self.get_observation()
        if robot_obs:
            with self.control_state_lock:
                self.oculus_initial_pose = np.concatenate([oculus_pos, oculus_quat])
                self.robot_initial_pose = robot_obs['pose_6d']
                self.control_active = True
                self.control_paused = False

            self.logger.info("🎯 控制已激活 - 初始位姿已记录")
            self.logger.info(f"   Oculus初始: pos={oculus_pos}, quat={oculus_quat}")
            self.logger.info(f"   机械臂初始: {self.robot_initial_pose}")
        else:
            self.logger.error("❌ 无法获取机械臂初始位姿，控制启动失败")

    def _pause_control(self):
        """暂停控制（保留初始位姿，可恢复）"""
        with self.control_state_lock:
            self.control_paused = True
        self.logger.info("⏸️  控制已暂停（按A恢复）")

    def _resume_control(self):
        """恢复控制"""
        with self.control_state_lock:
            self.control_paused = False
        self.logger.info("▶️  控制已恢复")

    def _stop_control(self):
        """停止控制（清除所有状态）"""
        with self.control_state_lock:
            self.control_active = False
            self.control_paused = False
            self.oculus_initial_pose = None
            self.robot_initial_pose = None
        self.logger.info("⏹️  控制已停止")

    def _save_and_stop(self):
        """保存数据并彻底结束控制"""
        # 先保存数据
        with self.trajectory_data_lock:
            data_count = len(self.trajectory_data)

        if self.enable_recording and data_count > 0:
            self.save()
        else:
            self.logger.warning("⚠️  没有数据可保存")

        # 然后停止控制
        self._stop_control()
        self.logger.info("💾 数据已保存，控制已结束")# ==================== 演示函数 ====================
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
