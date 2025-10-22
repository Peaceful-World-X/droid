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
import time
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
    from franky import Robot  # 机器人类，主要的机器人控制接口，用于连接和控制 Franka 机器人
    from franky import Twist  # 扭转类，用于表示空间中的速度（线速度和角速度）
    from franky import RelativeDynamicsFactor
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

    def __init__(self, host: Optional[str] = None, log_output: str = "both"):
        """初始化Franka R7控制器"""

        # 读取配置（从根配置文件中取 franka_r7arm 节），并允许通过构造函数覆盖 host
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            cfg_root = yaml.safe_load(f) or {}
            self.config = cfg_root.get('franka_r7arm', {})
        if host:
            self.config['host'] = host

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
            self.robot = Robot(self.config.get('host'))
            self.robot.recover_from_errors()
            self.robot.relative_dynamics_factor = RelativeDynamicsFactor(
                velocity=0.1, acceleration=0.05, jerk=0.1
            )
            self.logger.info("✅ Franka 机械臂连接成功！")

            # 连接夹爪
            self.gripper = RobotiqGripper()
            self.gripper.activate()
            if self.gripper.isActivated():
                self.gripper.calibrate(0, 100)
                self.logger.info("✅ Robotiq 夹爪连接成功！")
            else:
                self.logger.info("❌ Robotiq 夹爪连接失败！")

            # 设置连接状态
            self.is_connected = True
            
            # 机械臂回到初始位置
            self.return_to_initial_position()
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
    def get_current_status(self, print_flag=0) -> Optional[Dict[str, any]]:
        """获取机器人当前状态：
            - 末端位姿
            - 末端速度
            - 关节位置
            - 关节速度
            - 夹爪位置
        """
        try:
            # 末端位姿
            cartesian_state = self.robot.current_cartesian_state
            ee_pose: Affine = cartesian_state.pose.end_effector_pose
            trans = List[float] = ee_pose.translation   # [x, y, z]
            quat = List[float] = ee_pose.quaternion  # [x, y, z, w]
            euler = Rotation.from_quat(ee_pose.quaternion).as_euler('xyz')  # [roll, pitch, yaw] in rad
            ee_pose_6d = [*trans, *euler]

            # 末端速度
            ee_twist: Twist = cartesian_state.velocity.end_effector_twist
            linear_vel: List[float] = ee_twist.linear   # [vx, vy, vz] in m/s
            angular_vel: List[float] = ee_twist.angular # [wx, wy, wz] in rad/s
            ee_twist_6d = [*linear_vel, *angular_vel]

            # 关节状态
            joint_state = self.robot.current_joint_state
            joint_positions: List[float] = joint_state.position
            joint_velocities: List[float] = joint_state.velocity

            # 夹爪位置
            gripper_position = None
            if self.gripper:
                gripper_position = self.gripper.getPositionmm()/100.0

            # 状态字典
            status =  {
                "ee_pose": ee_pose_6d,
                "ee_velocity": ee_twist_6d,
                "joint_positions": joint_positions,
                "joint_velocities": joint_velocities,
                "gripper_position": gripper_position,
            }
            if print_flag==1:
                print(f"末端位姿: x ={trans[0]:6.3f} , y ={trans[1]:6.3f} , z ={trans[2]:6.3f} , "
                      f"qx={quat[0]:6.3f} , qy={quat[1]:6.3f} , qz={quat[2]:6.3f} , qw={quat[3]:6.3f}")
                print(f"末端速度: vx={linear_vel[0]:6.3f} , vy={linear_vel[1]:6.3f} , vz={linear_vel[2]:6.3f} , "
                      f"wx={angular_vel[0]:6.3f} , wy={angular_vel[1]:6.3f} , wz={angular_vel[2]:6.3f}")
                joints_pos = [f"J{i+1}={angle:6.3f}" for i, angle in enumerate(joint_positions)]
                print(f"关节位置: {' , '.join(joints_pos)}")
                joints_vel = [f"J{i+1}={vel:6.3f}" for i, vel in enumerate(joint_velocities)]
                print(f"关节速度: {' , '.join(joints_vel)}")
                print(f"夹爪张开: {status['gripper_position']*100:.2f}%")

            elif print_flag==2:
                print(f"末端位姿: x = {trans[0]:6.3f} , y = {trans[1]:6.3f} , z = {trans[2]:6.3f} , "
                      f"qx= {quat[0]:6.3f} , qy= {quat[1]:6.3f} , qz= {quat[2]:6.3f} , qw= {quat[3]:6.3f}")
                print(f"末端速度: vx= {linear_vel[0]:6.3f} , vy={linear_vel[1]:6.3f} , vz={linear_vel[2]:6.3f} , "
                      f"wx= {math.degrees(angular_vel[0]):6.1f} , wy= {math.degrees(angular_vel[1]):6.1f} , wz= {math.degrees(angular_vel[2]):6.1f}")
                joints_pos = [f"J{i+1}= {math.degrees(angle):6.1f}" for i, angle in enumerate(joint_positions)]
                print(f"关节位置: {' , '.join(joints_pos)}")
                joints_vel = [f"J{i+1}= {math.degrees(vel):6.1f}" for i, vel in enumerate(joint_velocities)]
                print(f"关节速度: {' , '.join(joints_vel)}")
                print(f"夹爪张开: {status['gripper_position']*100:6.1f}%")
            return status

        except Exception as e:
            self.logger.error(f"❌ 获取机器人状态失败: {e}")
            return None

    @connection_required
    def return_to_initial_position(self, target_position: Optional[List[float]] = None) -> bool:
        """回到初始位置或指定位置"""
        if self.gripper:
            self.gripper.open()
        try:
            target_radians = self.robot_init_joint if target_position is None else target_position
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

            ref_type = ReferenceType.Absolute if is_absolute else ReferenceType.Relative
            self.logger.info(f"🎯 发送笛卡尔位姿（{'绝对' if is_absolute else '相对'}）: 位置={position}, 姿态={orientation}")
            motion = CartesianMotion(affine, ref_type)
            self.robot.move(motion)
            return True

        except Exception as e:
            self.logger.error(f"❌ 发送笛卡尔位姿失败: {e}")
            return False

    @connection_required
    def send_gripper_position(self, position: float = 1.0, speed: float = 1.0, force: float = 1.0) -> bool:
        """控制夹爪位置（阻塞）
        Args:
            position: 夹爪开合程度，0.0（全闭）~ 1.0（全开）
            speed: 运动速度，0.0 ~ 1.0
            force: 夹持力，0.0 ~ 1.0
        """
        try:
            pos_cmd = max(0, min(100, int(position * 100)))
            spd_cmd = max(0, min(255, int(speed * 255)))
            frc_cmd = max(0, min(255, int(force * 255)))
            self.gripper.goTomm(pos_cmd, spd_cmd, frc_cmd)
            return True
        except Exception as e:
            self.logger.error(f"❌ 夹爪控制失败: {e}")
            return False

# ==================== 演示函数 ====================
def demo_basic_usage():
    """函数基本使用演示"""
    print("=" * 60)
    print("🤖 Franka R7 函数基本使用演示")
    print("=" * 60)

    # 创建控制器
    franka = FrankaR7arm()
    if not franka.connect():
        print("❌ 连接失败，演示结束")
        return

    FLAG = 0

    try:
        print("当前状态")
        franka.return_to_initial_position()
        franka.get_current_status(1)

        if FLAG==1:
            # 末端位姿绝对控制
            current_pose = franka.get_current_status()['ee_pose']
            current_pos = current_pose.translation
            current_rot = Rotation.from_quat(current_pose.quaternion).as_euler('xyz')
            target_pos = [current_pos[0] + 0.5, current_pos[1], current_pos[2]]
            target_pose = [*target_pos, *current_rot]
            franka.send_cartesian_pose(target_pose)
            time.sleep(2)
            print("✅ 末端位姿绝对控制完成")
            franka.get_current_status(1)

        elif FLAG==2:
            # 末端位姿相对控制
            relative_motion = [-0.05, 0, 0, 0, 0, 0]
            franka.send_cartesian_pose(relative_motion, is_absolute=False)
            time.sleep(2)
            print("✅ 末端位姿相对控制完成")
            franka.get_current_status(1)

        elif FLAG==3:
            # 角度绝对控制：最后一个关节旋转到-90度
            target_joints = franka.get_current_status()['joint_positions'].copy()
            target_joints[6] = target_joints[6]-math.pi/2
            franka.send_joint_angles(target_joints)
            time.sleep(2)
            print("✅ 关节角度控制完成")
            franka.get_current_status(1)

        print("当前状态")
        franka.return_to_initial_position()
        franka.get_current_status(1)
        pass

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
