# -*- encoding: utf-8 -*-
'''
@File    :   robosuite_r7arm.py
@Time    :   2025/09/29 15:38:16
@Author  :   Peaceful_World
@Version :   V1.0
@Contact :   Peaceful_World@qq.com
@Description: Robosuite仿真模式下的 Franka 机器人和夹爪的模拟实现
'''

from typing import List, Optional, Tuple
from enum import Enum
import time

# ================== 工具类 ==================
class ReferenceType(Enum):
    """参考系类型枚举"""
    Absolute = "absolute"
    Relative = "relative"

class Duration:
    """持续时间类，用于指定运动的时间长度"""

    def __init__(self, milliseconds: int):
        self.milliseconds = milliseconds
        self.seconds = milliseconds / 1000.0

class Twist:
    """扭转类，用于表示空间中的速度（线速度和角速度）"""

    def __init__(self, linear: List[float], angular: List[float]):
        self.linear = linear    # [vx, vy, vz] in m/s
        self.angular = angular  # [wx, wy, wz] in rad/s

class Affine:
    """仿射变换类，用于表示机器人的位置和姿态（包括平移和旋转）"""

    def __init__(self, position: List[float], quaternion: List[float]):
        self.position = position        # [x, y, z] 位置
        self.quaternion = quaternion    # [x, y, z, w] 四元数姿态

# ================== 状态类 ==================
class CartesianState:
    """笛卡尔状态类"""

    def __init__(self):
        self.pose = CartesianPose()
        self.velocity = CartesianVelocity()

class CartesianPose:
    """笛卡尔位姿类"""

    def __init__(self):
        # 默认位姿
        self.end_effector_pose = Affine([0.5, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0])

class CartesianVelocity:
    """笛卡尔速度类"""

    def __init__(self):
        # 默认速度为零
        self.end_effector_twist = Twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

class JointState:
    """关节状态类"""

    def __init__(self):
        # 7个关节的默认位置和速度
        self.position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# ================== 运动类 ==================
class CartesianMotion:
    """笛卡尔空间运动类，用于定义机器人末端执行器在笛卡尔坐标系中的运动"""

    def __init__(self, affine: Affine, reference_type: ReferenceType = ReferenceType.Absolute):
        self.affine = affine
        self.reference_type = reference_type

class CartesianVelocityMotion:
    """笛卡尔速度运动类，用于控制机器人末端执行器在笛卡尔空间的速度"""

    def __init__(self, twist: Twist, duration: Optional[Duration] = None):
        self.twist = twist
        self.duration = duration

class JointMotion:
    """关节运动类，用于定义机器人各关节的运动（关节空间运动）"""

    def __init__(self, joint_angles: List[float]):
        self.joint_angles = joint_angles

class JointStopMotion:
    """关节停止运动类，用于让机器人关节平滑停止运动"""

    def __init__(self):
        pass

class JointVelocityMotion:
    """关节速度运动类，用于控制机器人各关节的速度"""

    def __init__(self, velocities: List[float], duration: Optional[Duration] = None):
        self.velocities = velocities
        self.duration = duration

# ================== 核心类 ==================
class Robot:
    """机器人类，主要的机器人控制接口，用于连接和控制 Franka 机器人"""

    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip
        self.relative_dynamics_factor = 0.05
        self._current_joint_state = JointState()
        self._current_cartesian_state = CartesianState()
        print(f"🎭 仿真模式: 模拟连接到机器人 {robot_ip}")

    def recover_from_errors(self):
        """从错误中恢复"""
        print("🎭 仿真模式: 模拟错误恢复")
        time.sleep(0.1)  # 模拟恢复时间

    def move(self, motion):
        """执行运动命令"""
        motion_type = type(motion).__name__
        print(f"🎭 仿真模式: 执行 {motion_type}")

        if isinstance(motion, JointMotion):
            self._current_joint_state.position = motion.joint_angles
            print(f"    关节位置: {motion.joint_angles}")
        elif isinstance(motion, JointVelocityMotion):
            print(f"    关节速度: {motion.velocities}")
            if motion.duration:
                print(f"    持续时间: {motion.duration.seconds}s")
        elif isinstance(motion, CartesianMotion):
            print(f"    笛卡尔位置: {motion.affine.position}")
            print(f"    参考系: {motion.reference_type.value}")
        elif isinstance(motion, CartesianVelocityMotion):
            print(f"    线速度: {motion.twist.linear}")
            print(f"    角速度: {motion.twist.angular}")
            if motion.duration:
                print(f"    持续时间: {motion.duration.seconds}s")
        elif isinstance(motion, JointStopMotion):
            print("    停止所有关节运动")

        # 模拟运动时间
        time.sleep(0.1)

    @property
    def current_joint_state(self) -> JointState:
        """获取当前关节状态"""
        return self._current_joint_state

    @property
    def current_cartesian_state(self) -> CartesianState:
        """获取当前笛卡尔状态"""
        return self._current_cartesian_state

class RobotiqGripper:
    """Robotiq 夹爪仿真类"""

    def __init__(self):
        self._position = 255  # 初始位置（全开）
        self._is_activated = False
        print("🎭 仿真模式: 模拟连接到 Robotiq 夹爪")

    def activate(self) -> bool:
        """激活夹爪"""
        print("🎭 仿真模式: 激活夹爪")
        self._is_activated = True
        time.sleep(0.5)  # 模拟激活时间
        return True

    def open(self):
        """打开夹爪"""
        print("🎭 仿真模式: 打开夹爪")
        self._position = 255
        time.sleep(0.5)

    def close(self):
        """关闭夹爪"""
        print("🎭 仿真模式: 关闭夹爪")
        self._position = 0
        time.sleep(0.5)

    def goTo(self, position: int, speed: int = 255, force: int = 255) -> Tuple[int, bool]:
        """移动到指定位置
        Args:
            position: 目标位置 (0-255)
            speed: 运动速度 (0-255)
            force: 夹持力 (0-255)
        Returns:
            (actual_position, object_detected): 实际位置和是否检测到物体
        """
        print(f"🎭 仿真模式: 夹爪移动到位置 {position}, 速度 {speed}, 力度 {force}")
        self._position = max(0, min(255, position))
        time.sleep(0.3)  # 模拟运动时间

        # 模拟物体检测（随机）
        object_detected = position < 100  # 如果位置较小，假设检测到物体

        return self._position, object_detected

    def getPosition(self) -> int:
        """获取当前位置"""
        return self._position
