#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Franka R7 Robot Controller
    This module provides a unified controller for connecting, initializing, and controlling the Franka R7 robotic arm.

    - ENABLE_REAL = True: 真实机器人控制模式
    - ENABLE_REAL = False: 仿真模式（用于调试，无需真实硬件连接）

    Date: 2025-09-29
    Author: Xue Wenyao
"""
import math
import socket
from functools import wraps
from typing import Dict, List, Optional, Tuple

import yaml
from common import setup_logger
from scipy.spatial.transform import Rotation

# ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
ENABLE_REAL = socket.gethostname() == "cytoderm"
if ENABLE_REAL:
    print(f"🎯 使用真实模式")
    from franky import Affine                   # 仿射变换类，用于表示机器人的位置和姿态（包括平移和旋转）
    from franky import CartesianMotion          # 笛卡尔空间运动类，用于定义机器人末端执行器在笛卡尔坐标系中的运动
    from franky import CartesianVelocityMotion  # 笛卡尔速度运动类，用于控制机器人末端执行器在笛卡尔空间的速度
    from franky import Duration                 # 持续时间类，用于指定运动的时间长度
    from franky import JointMotion              # 关节运动类，用于定义机器人各关节的运动（关节空间运动）
    from franky import JointStopMotion          # 关节停止运动类，用于让机器人关节平滑停止运动
    from franky import JointVelocityMotion      # 关节速度运动类，用于控制机器人各关节的速度
    from franky import ReferenceType            # 参考系类型枚举，用于指定运动参考坐标系（如世界坐标系或末端执行器坐标系）
    from franky import Robot                    # 机器人类，主要的机器人控制接口，用于连接和控制 Franka 机器人
    from franky import Twist                    # 扭转类，用于表示空间中的速度（线速度和角速度）
    from pyrobotiqgripper import RobotiqGripper
else:
    print(f"🎭 使用仿真模式")
    from simu.robosuite_r7arm import *
    from simu.robosuite_r7arm import RobotiqGripper
# ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

def connection_required(func):
    """装饰器：检查机器人机械臂和夹爪的连接状态"""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_connected or not self.robot or not self.gripper:
            self.logger.error("❌ 未连接到机器人")
            return False if func.__name__.startswith(('send_', 'return_', 'stop_')) else None
        return func(self, *args, **kwargs)
    return wrapper

class FrankaR7arm:
    """Franka R7 机器人控制器,提供7轴机械臂和夹爪的统一控制接口，支持：
        - 关节位置和速度控制
        - 笛卡尔位姿和速度控制
        - 状态获取和监控
    """

    def __init__(self, log_output = "both"):
        """初始化Franka R7控制器"""

        self.config = self._load_config()
        self.logger = setup_logger("franka_r7arm", output=log_output)

        self.robot: Optional[Robot] = None
        self.gripper: Optional[RobotiqGripper] = None
        self.is_connected = False

        self.robot_init_joint = self.config.get('init_joint_angles', [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.robot_position_ranges = self.config.get('position_ranges', {
            'x': [0.1, 0.8],
            'y': [-0.5, 0.5],
            'z': [0.1, 0.8]
        })

    def _load_config(self) -> Dict:
        """从 yaml 文件加载配置 """
        config_path = "config.yaml"
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                return config.get('franka_r7arm', {})
        except (FileNotFoundError, yaml.YAMLError) as e:
            self.logger.error(f"⚠️ 加载配置文件失败: {e}，使用默认配置")
            return {}

    def _is_position_valid(self, position: List[float]) -> bool:
        """检查笛卡尔位置是否在工作空间内"""
        if len(position) < 3:
            return False
        x, y, z = position[:3]
        coords = {'x': x, 'y': y, 'z': z}
        for axis, value in coords.items():
            low, high = self.robot_position_ranges[axis]
            if not (low <= value <= high):
                self.logger.warning(f"⚠️ {axis.upper()}坐标 {value:.3f} 超出范围 [{low}, {high}]")
                return False
        return True

    def connect(self) -> bool:
        """连接到机器人和夹爪"""
        try:
            # 连接机器人
            self.robot = Robot(self.config['franka_r7arm']["host"])
            self.robot.recover_from_errors()
            self.robot.relative_dynamics_factor = 0.05
            self.logger.info("✅ Franka 机械臂连接成功！")

            # 连接夹爪
            self.gripper = RobotiqGripper()
            if self.gripper.activate():
                self.logger.info("✅ Robotiq 夹爪连接成功！")

            self.is_connected = True
            return True

        except Exception as e:
            self.logger.error(f"❌ 连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开连接"""
        self.gripper = None
        self.robot = None
        self.is_connected = False
        self.logger.info("🔌 机器人已断开连接")

    # =============== 运动控制 ===============
    @connection_required
    def get_current_status(self) -> Optional[Dict[str, any]]:
        """获取机器人当前状态：
            - 末端位姿 (Affine)
            - 末端速度 (Twist: linear + angular)
            - 关节位置/速度
            - 夹爪位置
        """

        try:
            # 末端位姿
            cartesian_state = self.robot.current_cartesian_state
            ee_pose: Affine = cartesian_state.pose.end_effector_pose

            # 末端速度
            ee_twist: Twist = cartesian_state.velocity.end_effector_twist
            linear_vel: List[float] = ee_twist.linear   # [vx, vy, vz] in m/s
            angular_vel: List[float] = ee_twist.angular # [wx, wy, wz] in rad/s

            # 关节状态
            joint_state = self.robot.current_joint_state
            joint_positions: List[float] = joint_state.position
            joint_velocities: List[float] = joint_state.velocity

            # 夹爪位置
            gripper_position = None
            if self.gripper:
                gripper_position = self.gripper.getPosition()/255

            return {
                "ee_pose": ee_pose,
                "ee_velocity": linear_vel + angular_vel,  # 6D: [vx, vy, vz, wx, wy, wz]
                "joint_positions": joint_positions,
                "joint_velocities": joint_velocities,
                "gripper_position": gripper_position,
            }

        except Exception as e:
            self.logger.error(f"❌ 获取机器人状态失败: {e}")
            return None

    @connection_required
    def return_to_initial_position(self, target_position: Optional[List[float]] = None, is_degrees: bool = False) -> bool:
        """回到初始位置或指定位置"""

        if self.gripper:
            self.gripper.open()
        try:
            target_radians = (
                self.robot_init_joint if target_position is None
                else [math.radians(d) for d in target_position] if is_degrees
                else target_position
            )
            self.logger.info(f"🏠 移动到初始位置: {[f'{math.degrees(q):.1f}°' for q in target_radians]}")
            self.robot.move(JointMotion(target_radians))
            return True
        except Exception as e:
            self.logger.error(f"❌ 回到初始位置失败: {e}")
            return False

    @connection_required
    def stop_execution(self) -> bool:
        """停止机器人执行"""
        try:
            self.robot.move(JointStopMotion())
            self.logger.info("✅ 机器人已停止")
            return True
        except Exception as e:
            self.logger.error(f"❌ 停止机器人失败: {e}")
            return False

    @connection_required
    def send_joint_angles(self, angles: List[float], is_degrees: bool = False) -> bool:
        """发送关节角度命令（绝对位置控制）
        Args:
            angles: 关节角度列表
            is_degrees: True表示angles是角度（度），False表示是弧度（默认）
        """
        try:
            target_radians = [math.radians(deg) for deg in angles] if is_degrees else angles
            self.logger.info(f"🎮 发送关节角度: {[f'{math.degrees(q):.1f}°' for q in target_radians]}")
            self.robot.move(JointMotion(target_radians))
            return True
        except Exception as e:
            self.logger.error(f"❌ 发送关节角度失败: {e}")
            return False

    @connection_required
    def send_joint_velocities(self, velocities: List[float], duration_ms: int = 1000) -> bool:
        """发送关节速度命令（速度控制）
        Args:
            velocities: 关节速度列表 (rad/s)
            duration_ms: 持续时间（毫秒）
        """
        try:
            self.logger.info(f"🎮 发送关节速度: {[f'{v:.3f}' for v in velocities]}, 持续: {duration_ms}ms")
            motion = JointVelocityMotion(velocities, duration=Duration(duration_ms))
            self.robot.move(motion)
            return True
        except Exception as e:
            self.logger.error(f"❌ 发送关节速度失败: {e}")
            return False

    @connection_required
    def send_cartesian_pose(self, pose_6d: List[float], is_absolute: bool = True) -> bool:
        """发送笛卡尔位姿命令（位置控制）
        Args:
            pose_6d: [x, y, z, roll, pitch, yaw] 笛卡尔位姿
            is_absolute: True 是绝对，False 是相对
        """
        try:
            ref_type = ReferenceType.Absolute if is_absolute  else ReferenceType.Relative
            position = pose_6d[:3]
            orientation = pose_6d[3:6] if len(pose_6d) >= 6 else [0.0, 0.0, 0.0]
            quat = Rotation.from_euler("xyz", orientation).as_quat()
            affine = Affine(position, quat)
            self.logger.info(f"🎯 发送笛卡尔位姿: 位置={position}, 姿态={[f'{math.degrees(o):.1f}°' for o in orientation]}")

            motion = CartesianMotion(affine, ref_type)
            self.robot.move(motion)
            return True

        except Exception as e:
            self.logger.error(f"❌ 发送笛卡尔位姿失败: {e}")
            return False

    @connection_required
    def send_cartesian_velocity(self, velocity_6d: List[float], duration_ms: int = 1000) -> bool:
        """发送笛卡尔速度命令（速度控制）
        Args:
            velocity_6d: [vx, vy, vz, wx, wy, wz] 笛卡尔速度
            duration_ms: 持续时间（毫秒）
        """
        try:
            linear_vel = velocity_6d[:3]
            angular_vel = velocity_6d[3:6] if len(velocity_6d) >= 6 else [0.0, 0.0, 0.0]
            self.logger.info(f"🎯 发送笛卡尔速度: 线速度={linear_vel}, 角速度={angular_vel}, 持续: {duration_ms}ms")

            motion = CartesianVelocityMotion(Twist(linear_vel, angular_vel), duration=Duration(duration_ms))
            self.robot.move(motion)
            return True

        except Exception as e:
            self.logger.error(f"❌ 发送笛卡尔速度失败: {e}")
            return False

    @connection_required
    def send_gripper_position(self, position: float = 1.0, speed: float = 1.0, force: float = 1.0) -> Tuple[Optional[float], Optional[bool]]:
        """控制夹爪位置（阻塞）
        Args:
            position: 夹爪开合程度，0.0（全闭）~ 1.0（全开）
            speed: 运动速度，0.0 ~ 1.0
            force: 夹持力，0.0 ~ 1.0
        Returns:
            - actual_position: 实际到达的位置（0~255），失败时为 None
            - object_detected: 是否检测到物体，失败时为 None
        """
        try:
            pos_cmd = max(0, min(255, int(position * 255)))
            spd_cmd = max(0, min(255, int(speed * 255)))
            frc_cmd = max(0, min(255, int(force * 255)))
            actual_position, object_detected = self.gripper.goTo(pos_cmd, spd_cmd, frc_cmd)
            return actual_position/255, object_detected
        except Exception as e:
            self.logger.error(f"❌ 夹爪控制失败: {e}")
            return None, None

# ==================== 演示函数 ====================
def demo_basic_usage():
    """基本使用演示"""
    print("=" * 60)
    print("🤖 Franka R7 基本使用演示")
    print("=" * 60)

    # 创建控制器
    robot_ip = "10.0.10.2" if ENABLE_REAL else "10.90.90.1"
    franka = FrankaR7arm(robot_ip)

    if not franka.connect():
        print("❌ 连接失败，演示结束")
        return

    try:
        # 获取初始状态
        print("\n📊 获取当前状态:")
        franka.get_status()

        # 回到初始位置
        print("\n🏠 回到初始位置:")
        franka.return_to_initial_position()

        # 关节位置控制演示
        print("\n🎮 关节位置控制演示:")
        test_angles = [0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]  # 度
        franka.send_joint_angles(test_angles, velocity_factor=0.1, is_degrees=True)

        # 关节速度控制演示
        print("\n🎮 关节速度控制演示:")
        test_velocities = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # rad/s
        franka.send_joint_velocities(test_velocities, duration_ms=2000)

        # 笛卡尔位置控制演示
        print("\n🎯 笛卡尔位置控制演示:")
        test_pose = [0.4, 0.0, 0.4, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
        franka.send_cartesian_pose(test_pose, velocity_factor=0.1)

        # 笛卡尔速度控制演示
        print("\n🎯 笛卡尔速度控制演示:")
        test_velocity = [0.05, 0.0, 0.0, 0.0, 0.0, 0.1]  # [vx, vy, vz, wx, wy, wz]
        franka.send_cartesian_velocity(test_velocity, duration_ms=2000)

        # 回到初始位置
        print("\n🏠 最后回到初始位置:")
        franka.return_to_initial_position()

        print("\n✅ 演示完成！")

    except KeyboardInterrupt:
        print("\n⏹️ 演示被用户中断")
        franka.stop_execution()
    except Exception as e:
        print(f"\n❌ 演示过程中出错: {e}")
    finally:
        franka.disconnect()

def main():
    try:
        demo_basic_usage()
    except Exception as e:
        print(f"❌ 程序异常: {e}")
    return 0

if __name__ == "__main__":
    main()
