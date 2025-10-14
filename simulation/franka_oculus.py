#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Oculus手柄实时控制Franka机械臂
结合Oculus手柄读取器和机械臂控制功能，实现3种控制模式：
1. 笛卡尔绝对位姿控制
2. 笛卡尔相对位姿控制（增量控制）
3. 关节空间控制

功能特性：
- 多线程实时控制（默认30Hz）
- 按钮映射到夹爪和功能控制
- 完整数据记录和回放
- 安全限制和平滑滤波
- 坐标系自动转换

Date: 2025-01-14
Author: Based on droid project
"""

import sys
import os
import time
import threading
import numpy as np
from pathlib import Path
from typing import Dict, Optional, Tuple, List
import json
from collections import deque
from scipy.spatial.transform import Rotation as R

# 添加路径
droid_path = Path(__file__).parent.parent / "droid"
sys.path.insert(0, str(droid_path))
sys.path.insert(0, str(droid_path / "oculus_reader" / "oculus_reader"))

# 导入Oculus读取器
from reader import OculusReader # ig

# 导入机械臂控制器（当前目录）
from franka_r7arm import FrankaR7arm


class SimpleController:
    """简化的控制命令转换器，集成在主文件中"""

    def __init__(self,
                 scale_factor: float = 1.0,
                 offset: List[float] = [0.4, 0.0, 0.3],
                 workspace_limits: Optional[Dict] = None,
                 velocity_limits: Optional[Dict] = None,
                 enable_smoothing: bool = True,
                 smoothing_window: int = 5,
                 deadzone: float = 0.002,
                 enable_recording: bool = True):

        self.scale_factor = scale_factor
        self.offset = np.array(offset)
        self.workspace_limits = workspace_limits or {
            'x': [0.2, 0.8],
            'y': [-0.4, 0.4],
            'z': [0.1, 0.8]
        }
        self.velocity_limits = velocity_limits or {
            'linear': 0.2,
            'angular': 0.5
        }

        self.enable_smoothing = enable_smoothing
        self.smoothing_window = smoothing_window
        self.deadzone = deadzone
        self.enable_recording = enable_recording

        # 平滑滤波缓冲区
        self.position_buffer = deque(maxlen=smoothing_window)
        self.rotation_buffer = deque(maxlen=smoothing_window)

        # 记录数据
        self.recorded_trajectory = []

        # 上一次的位姿
        self.last_position = None
        self.last_rotation = None
        self.last_button_state = {}

    def get_full_control_command(self, oculus_data: Tuple,
                                 hand: str = 'r',
                                 control_mode: str = 'relative',
                                 rotation_format: str = 'quaternion') -> Dict:
        """
        从Oculus数据生成完整的机械臂控制命令

        Args:
            oculus_data: (transforms, buttons) Oculus数据元组
            hand: 使用哪只手 'l' 或 'r'
            control_mode: 'absolute', 'relative', 或 'joint'
            rotation_format: 'euler_xyz', 'quaternion', 或 'axis_angle'

        Returns:
            控制命令字典
        """
        transforms, buttons = oculus_data

        # 提取变换矩阵
        if hand not in transforms:
            return {'emergency_stop': True}

        transform_matrix = transforms[hand]

        # 提取位置和旋转
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]

        # 转换坐标
        position = self._transform_position(position)

        # 转换旋转
        rotation = self._convert_rotation(rotation_matrix, rotation_format)

        # 应用平滑
        if self.enable_smoothing:
            position = self._smooth_position(position)
            rotation = self._smooth_rotation(rotation, rotation_format)

        # 构建命令
        command = {
            'position': position.tolist(),
            'rotation': rotation.tolist() if isinstance(rotation, np.ndarray) else rotation,
            'control_mode': control_mode,
            'rotation_format': rotation_format,
            'timestamp': time.time()
        }

        # 添加相对控制的增量
        if control_mode == 'relative':
            if self.last_position is not None:
                delta_pos = position - self.last_position
                # 应用死区
                if np.linalg.norm(delta_pos) < self.deadzone:
                    delta_pos = np.zeros(3)
                command['delta_position'] = delta_pos.tolist()
            else:
                command['delta_position'] = [0, 0, 0]

        # 处理按钮
        button_actions = self._process_buttons(buttons)
        command.update(button_actions)

        # 记录
        if self.enable_recording:
            self.recorded_trajectory.append(command.copy())

        # 保存状态
        self.last_position = position.copy()
        self.last_rotation = rotation.copy() if isinstance(rotation, np.ndarray) else np.array(rotation)

        return command

    def _transform_position(self, position: np.ndarray) -> np.ndarray:
        """转换位置坐标"""
        # 缩放和偏移
        transformed = position * self.scale_factor + self.offset

        # 限制在工作空间内
        transformed[0] = np.clip(transformed[0], *self.workspace_limits['x'])
        transformed[1] = np.clip(transformed[1], *self.workspace_limits['y'])
        transformed[2] = np.clip(transformed[2], *self.workspace_limits['z'])

        return transformed

    def _convert_rotation(self, rotation_matrix: np.ndarray, format_type: str):
        """转换旋转表示"""
        rot = R.from_matrix(rotation_matrix)

        if format_type == 'euler_xyz':
            return rot.as_euler('xyz', degrees=False)
        elif format_type == 'quaternion':
            return rot.as_quat()  # [x, y, z, w]
        elif format_type == 'axis_angle':
            rotvec = rot.as_rotvec()
            angle = np.linalg.norm(rotvec)
            axis = rotvec / angle if angle > 1e-6 else np.array([0, 0, 1])
            return {'axis': axis.tolist(), 'angle': float(angle)}
        else:
            return rot.as_quat()

    def _smooth_position(self, position: np.ndarray) -> np.ndarray:
        """平滑位置"""
        self.position_buffer.append(position)
        if len(self.position_buffer) > 0:
            return np.mean(self.position_buffer, axis=0)
        return position

    def _smooth_rotation(self, rotation, format_type: str):
        """平滑旋转"""
        if format_type == 'quaternion':
            self.rotation_buffer.append(rotation)
            if len(self.rotation_buffer) > 1:
                # 四元数平均（简单方法）
                return np.mean(self.rotation_buffer, axis=0)
        return rotation

    def _process_buttons(self, buttons: Dict) -> Dict:
        """处理按钮输入"""
        actions = {}

        # A按钮 - 夹爪关闭
        if buttons.get('A', False) and not self.last_button_state.get('A', False):
            actions['gripper_position'] = 'close'

        # B按钮 - 夹爪打开
        elif buttons.get('B', False) and not self.last_button_state.get('B', False):
            actions['gripper_position'] = 'open'
        else:
            actions['gripper_position'] = None

        # 右摇杆按下 - 急停
        if buttons.get('RJ', False):
            actions['emergency_stop'] = True
        else:
            actions['emergency_stop'] = False

        # 右触发器 - 精确模式
        actions['precision_mode'] = buttons.get('RTr', 0.0) > 0.5

        self.last_button_state = buttons.copy()
        return actions

    def save_recording(self, filename: str):
        """保存记录的轨迹"""
        if not self.recorded_trajectory:
            return

        data = {
            'trajectory': self.recorded_trajectory,
            'metadata': {
                'num_points': len(self.recorded_trajectory),
                'scale_factor': self.scale_factor,
                'workspace_limits': self.workspace_limits,
                'timestamp': time.strftime("%Y-%m-%d %H:%M:%S")
            }
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)


class FrankaOculusController:
    """Franka机械臂的Oculus手柄控制器"""

    def __init__(self,
                 control_mode: str = 'relative',
                 control_frequency: float = 30.0,
                 enable_recording: bool = True,
                 robot_ip: Optional[str] = None,
                 oculus_ip: Optional[str] = None):
        """
        初始化控制器

        Args:
            control_mode: 控制模式 'absolute', 'relative', 或 'joint'
            control_frequency: 控制频率 (Hz)
            enable_recording: 是否启用数据记录
            robot_ip: 机械臂IP地址
            oculus_ip: Oculus设备IP地址
        """
        print("=" * 70)
        print("🤖 Franka Oculus 遥操作控制器")
        print("=" * 70)

        # 控制参数
        self.control_mode = control_mode
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency
        self.enable_recording = enable_recording

        # 初始化Oculus读取器
        print("\n📱 初始化Oculus手柄读取器...")
        self.oculus_reader = OculusReader(ip_address=oculus_ip)

        # 初始化机械臂
        print("\n🦾 初始化Franka机械臂...")
        self.robot = FrankaR7arm()
        if not self.robot.connect():
            raise RuntimeError("无法连接到Franka机械臂")

        # 初始化控制转换器
        print("\n🔄 初始化控制转换器...")
        self.controller = SimpleController(
            scale_factor=1.0,
            offset=[0.4, 0.0, 0.3],
            workspace_limits={
                'x': [0.2, 0.8],
                'y': [-0.4, 0.4],
                'z': [0.1, 0.8]
            },
            velocity_limits={
                'linear': 0.2,
                'angular': 0.5
            },
            enable_smoothing=True,
            smoothing_window=5,
            deadzone=0.002,
            enable_recording=enable_recording
        )

        # 控制线程
        self.control_thread: Optional[threading.Thread] = None
        self.running = False
        self.paused = False

        # 统计信息
        self.stats = {
            'total_commands': 0,
            'successful_commands': 0,
            'failed_commands': 0,
            'start_time': None,
            'last_command_time': None,
            'actual_frequency': 0.0
        }

        # 夹爪状态
        self.last_gripper_state = None

        print("\n✅ 初始化完成！")
        print(f"   控制模式: {self.control_mode}")
        print(f"   控制频率: {self.control_frequency} Hz")
        print(f"   数据记录: {'启用' if self.enable_recording else '禁用'}")

    def _control_loop(self):
        """控制循环主函数（在独立线程中运行）"""
        print(f"\n🚀 启动控制循环 (频率: {self.control_frequency} Hz)")

        self.stats['start_time'] = time.time()
        last_print_time = time.time()
        command_count = 0

        while self.running:
            if self.paused:
                time.sleep(0.1)
                continue

            loop_start = time.time()

            try:
                # 获取Oculus手柄数据
                transforms, buttons = self.oculus_reader.get_transformations_and_buttons()

                if not transforms or 'r' not in transforms:
                    time.sleep(self.control_period)
                    continue

                oculus_data = (transforms, buttons)

                # 获取控制命令
                command = self.controller.get_full_control_command(
                    oculus_data,
                    hand='r',
                    control_mode=self.control_mode,
                    rotation_format='quaternion'
                )

                # 检查急停
                if command.get('emergency_stop'):
                    print("\n🛑 检测到急停信号！")
                    self.pause()
                    continue

                # 执行控制命令
                success = self._execute_command(command)

                # 更新统计
                self.stats['total_commands'] += 1
                if success:
                    self.stats['successful_commands'] += 1
                else:
                    self.stats['failed_commands'] += 1

                self.stats['last_command_time'] = time.time()
                command_count += 1

                # 每秒打印一次状态
                if time.time() - last_print_time >= 1.0:
                    elapsed = time.time() - self.stats['start_time']
                    actual_freq = command_count / (time.time() - last_print_time)
                    self.stats['actual_frequency'] = actual_freq

                    print(f"\r⏱️  运行: {elapsed:.1f}s | "
                          f"频率: {actual_freq:.1f}Hz | "
                          f"命令: {self.stats['total_commands']} | "
                          f"成功: {self.stats['successful_commands']} | "
                          f"失败: {self.stats['failed_commands']}",
                          end='', flush=True)

                    last_print_time = time.time()
                    command_count = 0

            except Exception as e:
                print(f"\n❌ 控制循环错误: {e}")
                self.stats['failed_commands'] += 1

            # 控制频率
            loop_time = time.time() - loop_start
            sleep_time = self.control_period - loop_time
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("\n\n🏁 控制循环已停止")

    def _execute_command(self, command: Dict) -> bool:
        """执行控制命令"""
        try:
            # 处理夹爪控制
            if 'gripper_position' in command and command['gripper_position'] is not None:
                self._control_gripper(command['gripper_position'])

            # 处理位姿控制
            if self.control_mode == 'relative' and 'delta_position' in command:
                return self._execute_relative_control(command)
            elif self.control_mode == 'absolute' and 'position' in command:
                return self._execute_absolute_control(command)

            return True

        except Exception as e:
            print(f"\n⚠️ 命令执行失败: {e}")
            return False

    def _execute_absolute_control(self, command: Dict) -> bool:
        """执行绝对位姿控制"""
        position = command['position']
        rotation = command['rotation']  # 四元数 [x, y, z, w]

        return self.robot.send_target_cartesian_pose(
            position, rotation, is_degrees=False
        )

    def _execute_relative_control(self, command: Dict) -> bool:
        """执行相对位姿控制（增量控制）"""
        delta_position = np.array(command['delta_position'])

        # 检查是否有足够的移动量
        if np.linalg.norm(delta_position) < self.controller.deadzone:
            return True

        # 应用精确模式缩放
        if command.get('precision_mode'):
            delta_position *= 0.3

        # 发送速度命令（将增量转换为速度）
        velocity = delta_position / self.control_period

        # 限制速度
        speed = np.linalg.norm(velocity)
        if speed > self.controller.velocity_limits['linear']:
            velocity = velocity * self.controller.velocity_limits['linear'] / speed

        # 发送笛卡尔速度命令 [vx, vy, vz, wx, wy, wz]
        return self.robot.send_target_cartesian_velocity(
            velocity.tolist() + [0, 0, 0]
        )

    def _control_gripper(self, gripper_command):
        """控制夹爪"""
        if gripper_command == self.last_gripper_state:
            return

        try:
            if gripper_command == 'open':
                self.robot.gripper.open()
                print("\n✋ 夹爪打开")
            elif gripper_command == 'close':
                self.robot.gripper.close()
                print("\n✊ 夹爪关闭")

            self.last_gripper_state = gripper_command

        except Exception as e:
            print(f"\n⚠️ 夹爪控制失败: {e}")

    def start(self):
        """启动控制"""
        if self.running:
            print("⚠️ 控制已在运行中")
            return

        print("\n" + "=" * 70)
        print("🎮 控制说明:")
        print("   • 移动右手柄控制机械臂末端位置")
        print("   • A按钮 - 夹爪关闭")
        print("   • B按钮 - 夹爪打开")
        print("   • 右触发器 - 精确模式（降低速度）")
        print("   • 右摇杆按下 - 急停")
        print("=" * 70)

        # 移动到初始位置
        print("\n🏠 移动到初始位置...")
        self.robot.return_to_initial_position()
        time.sleep(1.0)

        # 启动控制线程
        self.running = True
        self.paused = False
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        print("\n✅ 控制已启动！")

    def pause(self):
        """暂停控制"""
        if not self.running:
            print("⚠️ 控制未运行")
            return

        self.paused = True
        print("\n⏸️  控制已暂停")

    def resume(self):
        """恢复控制"""
        if not self.running:
            print("⚠️ 控制未运行")
            return

        self.paused = False
        print("\n▶️  控制已恢复")

    def stop(self):
        """停止控制"""
        if not self.running:
            return

        print("\n\n🛑 停止控制...")
        self.running = False

        if self.control_thread:
            self.control_thread.join(timeout=2.0)

        # 停止机器人运动
        self.robot.stop_execution()

        # 保存数据
        if self.enable_recording:
            self.save_recording()

        # 打印统计信息
        self.print_statistics()

        print("✅ 控制已停止")

    def save_recording(self, filename: Optional[str] = None):
        """保存记录的数据"""
        if not self.enable_recording:
            print("⚠️ 数据记录未启用")
            return

        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"franka_oculus_trajectory_{timestamp}.json"

        self.controller.save_recording(filename)
        print(f"💾 数据已保存到: {filename}")

    def print_statistics(self):
        """打印统计信息"""
        print("\n" + "=" * 70)
        print("📊 运行统计:")
        print("-" * 70)

        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0

        print(f"   总运行时间: {elapsed:.1f} 秒")
        print(f"   总命令数: {self.stats['total_commands']}")
        print(f"   成功命令: {self.stats['successful_commands']}")
        print(f"   失败命令: {self.stats['failed_commands']}")
        print(f"   成功率: {100 * self.stats['successful_commands'] / max(1, self.stats['total_commands']):.1f}%")
        print(f"   平均频率: {self.stats['total_commands'] / max(1, elapsed):.2f} Hz")
        print(f"   实际频率: {self.stats['actual_frequency']:.2f} Hz")

        print("=" * 70)

    def cleanup(self):
        """清理资源"""
        print("\n🧹 清理资源...")

        self.stop()

        # 断开机器人连接
        if self.robot:
            self.robot.disconnect()

        # 停止Oculus读取器
        if self.oculus_reader:
            self.oculus_reader.stop()

        print("✅ 清理完成")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='Franka机械臂Oculus遥操作控制')
    parser.add_argument('--mode', type=str, default='relative',
                       choices=['absolute', 'relative', 'joint'],
                       help='控制模式 (默认: relative)')
    parser.add_argument('--frequency', type=float, default=30.0,
                       help='控制频率 Hz (默认: 30)')
    parser.add_argument('--no-recording', action='store_true',
                       help='禁用数据记录')
    parser.add_argument('--robot-ip', type=str, default=None,
                       help='机械臂IP地址')
    parser.add_argument('--oculus-ip', type=str, default=None,
                       help='Oculus设备IP地址')

    args = parser.parse_args()

    # 创建控制器
    controller = None

    try:
        controller = FrankaOculusController(
            control_mode=args.mode,
            control_frequency=args.frequency,
            enable_recording=not args.no_recording,
            robot_ip=args.robot_ip,
            oculus_ip=args.oculus_ip
        )

        # 启动控制
        controller.start()

        # 运行直到用户中断
        print("\n💡 按 Ctrl+C 停止控制\n")

        while controller.running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n⚠️ 检测到键盘中断")

    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if controller:
            controller.cleanup()


if __name__ == "__main__":
    main()
