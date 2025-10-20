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
# ==================================================================
# ⚪白色:交互       Franka 可以安全操作
# 🔵蓝色:已激活     Franka 已启用移动功能，随时可以开始移动
# 🟢绿色:自动执行   Franka 正在执行自动程序并独立移动
# 🟡橙色:已锁定     Franka 以机械方式锁定或无法使用
# 🟣粉色:冲突       Franka 接到冲突启用信号
# 🔴红色:错误       Franka 发生错误
# ==================================================================
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
    from franky import Affine  # 仿射变换类，用于表示机器人的位置和姿态（包括平移和旋转）
    from franky import CartesianMotion  # 笛卡尔运动类，用于定义机器人末端执行器在笛卡尔坐标系中的运动
    from franky import JointMotion  # 关节运动类，用于定义机器人各关节的运动（关节空间运动）
    from franky import JointStopMotion  # 关节运动类，用于让机器人关节平滑停止运动
    from franky import ReferenceType  # 参考系类型枚举，用于指定运动参考坐标系（如世界坐标系或末端执行器坐标系）
    from franky import RelativeDynamicsFactor
    from franky import Robot  # 机器人类，主要的机器人控制接口，用于连接和控制 Franka 机器人
    from franky import Twist  # 扭转类，用于表示空间中的速度（线速度和角速度）
    from pyrobotiqgripper import RobotiqGripper
else:
    print(f"🎭 使用仿真模式")
    from simu.robosuite_franky import *
    from simu.robosuite_franky import RobotiqGripper
# ————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
CONFIG_PATH = "config.yaml"

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

        # 读取配置
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f).get('franka_r7arm', {})

        # 设置日志
        self.logger = setup_logger("franka_r7arm", output=log_output)

        # 设置机械臂、夹爪
        self.robot: Optional[Robot] = None
        self.gripper: Optional[RobotiqGripper] = None
        self.is_connected = False
        self.robot_init_joint = self.config.get('init_joint_angles', [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.robot_position_ranges = self.config.get('position_ranges', {'x': [0.1, 0.8], 'y': [-0.5, 0.5], 'z': [0.1, 0.8]})

    def connect(self) -> bool:
        """连接到机器人和夹爪"""
        try:
            # 连接机器人
            self.robot = Robot(self.config['franka_r7arm']["host"])
            self.robot.recover_from_errors()
            self.robot.relative_dynamics_factor = RelativeDynamicsFactor(
                velocity=0.1, acceleration=0.05, jerk=0.1
            )
            self.logger.info("✅ Franka 机械臂连接成功！")

            # 连接夹爪
            self.gripper = RobotiqGripper()
            if self.gripper.activate():
                self.logger.info("✅ Robotiq 夹爪连接成功！")
            else:
                self.logger.info("❌ Robotiq 夹爪连接失败！")

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
    def send_joint_angles(self, target_radians: List[float]) -> bool:
        """发送关节角度命令（绝对位置控制）
        """
        try:
            self.logger.info(f"🎮 发送关节角度: {target_radians}")
            self.robot.move(JointMotion(target_radians))
            return True
        except Exception as e:
            self.logger.error(f"❌ 发送关节角度失败: {e}")
            return False

    @connection_required
    def send_cartesian_pose(self, pose_6d: List[float], is_absolute: bool = True) -> bool:
        """发送笛卡尔位姿命令（位置控制）
        Args:
            pose_6d: [x, y, z, roll, pitch, yaw] 笛卡尔位姿
            is_absolute: True 是绝对，False 是相对
        """
        try:
            position = pose_6d[:3]
            orientation = pose_6d[3:6] if len(pose_6d) >= 6 else [0.0, 0.0, 0.0]
            affine = Affine(position, Rotation.from_euler("xyz", orientation).as_quat())

            ref_type = ReferenceType.Absolute if is_absolute  else ReferenceType.Relative
            self.logger.info(f"🎯 发送笛卡尔位姿（{"绝对" if is_absolute else "相对"}）: 位置={position}, 姿态={orientation}")
            motion = CartesianMotion(affine, ref_type)
            self.robot.move(motion)
            return True

        except Exception as e:
            self.logger.error(f"❌ 发送笛卡尔位姿失败: {e}")
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
    robot_ip = "10.0.10.2"
    franka = FrankaR7arm(robot_ip)

    if not franka.connect():
        print("❌ 连接失败，演示结束")
        return

    try:
        # 获取初始状态
        print("\n📊 获取当前状态:")
        status = franka.get_current_status()
        if status:
            print(f"末端位姿: {status['ee_pose']}")
            print(f"末端速度: {status['ee_velocity']}")
            print(f"关节位置: {status['joint_positions']}")
            print(f"关节速度: {status['joint_velocities']}")
            print(f"夹爪位置: {status['gripper_position']}")

    except KeyboardInterrupt:
        print("\n⏹️ 演示被用户中断")
        franka.stop_execution()
    except Exception as e:
        print(f"\n❌ 演示过程中出错: {e}")
    finally:
        franka.disconnect()

def main():
    demo_basic_usage()
    return 0

if __name__ == "__main__":
    main()
