#!/usr/bin/env python3
"""
Franka机器人控制演示脚本
演示如何使用FrankaRobot类进行基本的机器人控制操作
"""

import time

from droid.franka.robot import FrankaRobot


def demo_franka_control():
    """演示Franka机器人控制的完整流程"""
    # 初始化
    robot = FrankaRobot()
    robot.launch_controller()
    robot.launch_robot()

    try:
        # 获取当前状态
        current_pose = robot.get_ee_pose()
        print(f"当前位姿: {current_pose}")

        # 获取当前关节状态
        joint_positions = robot.get_joint_positions()
        print(f"当前关节位置: {joint_positions}")

        # 获取夹爪状态
        gripper_pos = robot.get_gripper_position()
        print(f"当前夹爪位置: {gripper_pos}")

        print("\n开始执行控制演示...")

        # 1. 笛卡尔位置控制 - Z轴上升
        print("1. 末端执行器上升10cm...")
        target_pose = current_pose.copy()
        target_pose[2] += 0.1  # Z轴上升10cm
        robot.update_pose(target_pose, blocking=True)
        print("   移动完成")
        time.sleep(1)

        # 2. 笛卡尔速度控制演示
        print("2. 笛卡尔速度控制 - X轴正方向移动...")
        velocity = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]  # X轴正方向5cm/s
        robot.update_pose(velocity, velocity=True, blocking=False)
        time.sleep(2)  # 移动2秒

        # 停止运动
        stop_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        robot.update_pose(stop_velocity, velocity=True, blocking=False)
        print("   速度控制完成")
        time.sleep(1)

        # 3. 夹爪控制演示
        print("3. 夹爪控制演示...")
        print("   夹爪闭合到50%...")
        robot.update_gripper(0.5, velocity=False, blocking=True)
        time.sleep(1)

        print("   夹爪张开...")
        robot.update_gripper(0.1, velocity=False, blocking=True)
        time.sleep(1)

        # 4. 统一接口控制演示
        print("4. 使用统一接口进行控制...")
        # 笛卡尔速度控制 + 夹爪控制
        command = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.2]  # Y轴移动 + 夹爪位置
        action_dict = robot.update_command(
            command,
            action_space="cartesian_velocity",
            gripper_action_space="velocity"
        )
        print(f"   执行动作: {action_dict['cartesian_velocity']}")
        time.sleep(2)

        # 停止运动
        stop_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        robot.update_command(stop_command, action_space="cartesian_velocity")

        # 5. 回到初始位置
        print("5. 回到初始位置...")
        robot.update_pose(current_pose, blocking=True)
        robot.update_gripper(gripper_pos, velocity=False, blocking=True)

        print("\n控制演示完成！")

        # 显示最终状态
        final_pose = robot.get_ee_pose()
        print(f"最终位姿: {final_pose}")

    except Exception as e:
        print(f"控制过程中出现错误: {e}")

    finally:
        # 清理资源
        print("清理资源...")
        robot.kill_controller()


def demo_advanced_control():
    """演示高级控制功能"""
    robot = FrankaRobot()
    robot.launch_controller()
    robot.launch_robot()

    try:
        print("高级控制功能演示...")

        # 1. 带噪声的关节控制
        print("1. 带笛卡尔噪声的关节控制...")
        joint_positions = robot.get_joint_positions()
        cartesian_noise = [0.005, 0.005, 0.005, 0.05, 0.05, 0.05]  # 小幅噪声
        robot.update_joints(joint_positions, cartesian_noise=cartesian_noise)
        time.sleep(2)

        # 2. 动作解析演示
        print("2. 动作解析功能演示...")
        action = [0.02, 0.0, 0.01, 0.0, 0.0, 0.05, 0.3]
        action_dict = robot.create_action_dict(
            action,
            action_space="cartesian_velocity",
            gripper_action_space="position"
        )

        print("解析后的动作:")
        for key, value in action_dict.items():
            if key != "robot_state":  # 跳过状态信息
                print(f"  {key}: {value}")

        # 3. 自适应时间计算
        print("3. 自适应运动时间计算...")
        current_joints = robot.get_joint_positions()
        target_joints = [j + 0.1 for j in current_joints]  # 小幅关节移动
        import torch
        time_to_go = robot.adaptive_time_to_go(torch.tensor(target_joints))
        print(f"   计算的运动时间: {time_to_go:.2f}秒")

    except Exception as e:
        print(f"高级控制演示出现错误: {e}")

    finally:
        robot.kill_controller()


if __name__ == "__main__":
    print("=== Franka机器人控制演示 ===\n")

    # 基础控制演示
    print("执行基础控制演示...")
    demo_franka_control()

    print("\n" + "="*50 + "\n")

    # 高级控制演示
    print("执行高级控制演示...")
    demo_advanced_control()

    print("\n演示程序结束。")