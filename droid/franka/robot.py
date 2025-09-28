# ROBOT SPECIFIC IMPORTS
import os
import time

import grpc
import numpy as np
import torch
from polymetis import GripperInterface, RobotInterface

from droid.misc.parameters import sudo_password
from droid.misc.subprocess_utils import run_terminal_command, run_threaded_command

# UTILITY SPECIFIC IMPORTS
from droid.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from droid.robot_ik.robot_ik_solver import RobotIKSolver


class FrankaRobot:
    """Franka 七轴机械臂控制封装，负责协调移动、夹爪与逆解求解器。"""

    def launch_controller(self):
        """启动机器人与夹爪控制后台，必要时先终止残留进程。"""
        try:
            self.kill_controller()
        except:
            pass

        dir_path = os.path.dirname(os.path.realpath(__file__))
        self._robot_process = run_terminal_command(
            "echo " + sudo_password + " | sudo -S " + "bash " + dir_path + "/launch_robot.sh"
        )
        self._gripper_process = run_terminal_command(
            "echo " + sudo_password + " | sudo -S " + "bash " + dir_path + "/launch_gripper.sh"
        )
        self._server_launched = True
        time.sleep(5)

    def launch_robot(self):
        """连接本地 Polymetis 服务并初始化 IK 求解器与夹爪参数。"""
        self._robot = RobotInterface(ip_address="localhost")
        self._gripper = GripperInterface(ip_address="localhost")
        self._max_gripper_width = self._gripper.metadata.max_width
        self._ik_solver = RobotIKSolver()
        self._controller_not_loaded = False

    def kill_controller(self):
        """终止机器人与夹爪的后台控制进程。"""
        self._robot_process.kill()
        self._gripper_process.kill()

    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        """解析高层动作并同步更新关节与夹爪。

        参数:
            command (Sequence[float]): 动作向量，末元素用于夹爪控制。
            action_space (str): 机器人动作空间，支持笛卡尔/关节的速度或位置模式。
            gripper_action_space (str, 可选): 夹爪控制方式，默认随机器人模式自动推断。
            blocking (bool): 是否阻塞等待动作执行完成。

        返回:
            dict: 包含笛卡尔、关节与夹爪的派生动作信息。
        """
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)

        self.update_joints(action_dict["joint_position"], velocity=False, blocking=blocking)
        self.update_gripper(action_dict["gripper_position"], velocity=False, blocking=blocking)

        return action_dict

    def update_pose(self, command, velocity=False, blocking=False):
        """根据笛卡尔位姿或速度对末端执行器进行控制。

        参数:
            command (Sequence[float]): 期望末端位姿 (xyz + rpy) 或笛卡尔速度。
            velocity (bool): 指示 command 是否表示速度。
            blocking (bool): 是否阻塞至动作完成。
        """
        if blocking:
            if velocity:
                curr_pose = self.get_ee_pose()
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(command)
                command = add_poses(cartesian_delta, curr_pose)

            pos = torch.Tensor(command[:3])
            quat = torch.Tensor(euler_to_quat(command[3:6]))
            curr_joints = self._robot.get_joint_positions()
            desired_joints = self._robot.solve_inverse_kinematics(pos, quat, curr_joints)
            self.update_joints(desired_joints, velocity=False, blocking=True)
        else:
            if not velocity:
                curr_pose = self.get_ee_pose()
                cartesian_delta = pose_diff(command, curr_pose)
                command = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)

            robot_state = self.get_robot_state()[0]
            joint_velocity = self._ik_solver.cartesian_velocity_to_joint_velocity(command, robot_state=robot_state)

            self.update_joints(joint_velocity, velocity=True, blocking=False)

    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """执行关节级控制，可选速度模式并支持笛卡尔噪声扰动。

        参数:
            command (Sequence[float]): 目标关节位置或速度向量。
            velocity (bool): 若为 True，则 command 表示速度并会积分到目标位置。
            blocking (bool): 若为 True，阻塞直至关节动作完成。
            cartesian_noise (Sequence[float], 可选): 叠加在末端位姿的噪声 (xyz + rpy)。
        """
        if cartesian_noise is not None:
            command = self.add_noise_to_joints(command, cartesian_noise)
        command = torch.Tensor(command)

        if velocity:
            joint_delta = self._ik_solver.joint_velocity_to_delta(command)
            command = joint_delta + self._robot.get_joint_positions()

        def helper_non_blocking():
            if not self._robot.is_running_policy():
                self._controller_not_loaded = True
                self._robot.start_cartesian_impedance()
                timeout = time.time() + 5
                while not self._robot.is_running_policy():
                    time.sleep(0.01)
                    if time.time() > timeout:
                        self._robot.start_cartesian_impedance()
                        timeout = time.time() + 5

                self._controller_not_loaded = False
            try:
                self._robot.update_desired_joint_positions(command)
            except grpc.RpcError:
                pass

        if blocking:
            if self._robot.is_running_policy():
                self._robot.terminate_current_policy()
            try:
                time_to_go = self.adaptive_time_to_go(command)
                self._robot.move_to_joint_positions(command, time_to_go=time_to_go)
            except grpc.RpcError:
                pass

            self._robot.start_cartesian_impedance()
        else:
            if not self._controller_not_loaded:
                run_threaded_command(helper_non_blocking)

    def update_gripper(self, command, velocity=True, blocking=False):
        """按速度或目标位置驱动夹爪，并自动裁剪到可行区间。

        参数:
            command (float): 夹爪速度或位置，范围约束在 [0,1]。
            velocity (bool): 若为 True，将 command 解释为速度增量。
            blocking (bool): 是否阻塞等待夹爪动作结束。
        """
        if velocity:
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            command = gripper_delta + self.get_gripper_position()

        command = float(np.clip(command, 0, 1))
        self._gripper.goto(width=self._max_gripper_width * (1 - command), speed=0.05, force=0.1, blocking=blocking)

    def add_noise_to_joints(self, original_joints, cartesian_noise):
        """在笛卡尔空间注入噪声并通过 IK 回算到关节空间。

        参数:
            original_joints (Sequence[float]): 当前的关节位置。
            cartesian_noise (Sequence[float]): 笛卡尔扰动 (xyz + rpy)。

        返回:
            list[float]: 噪声后的关节位置，若求解失败则回退原值。
        """
        original_joints = torch.Tensor(original_joints)

        pos, quat = self._robot.robot_model.forward_kinematics(original_joints)
        curr_pose = pos.tolist() + quat_to_euler(quat).tolist()
        new_pose = add_poses(cartesian_noise, curr_pose)

        new_pos = torch.Tensor(new_pose[:3])
        new_quat = torch.Tensor(euler_to_quat(new_pose[3:]))

        noisy_joints, success = self._robot.solve_inverse_kinematics(new_pos, new_quat, original_joints)

        if success:
            desired_joints = noisy_joints
        else:
            desired_joints = original_joints

        return desired_joints.tolist()

    def get_joint_positions(self):
        """获取当前关节位置。"""
        return self._robot.get_joint_positions().tolist()

    def get_joint_velocities(self):
        """获取当前关节速度。"""
        return self._robot.get_joint_velocities().tolist()

    def get_gripper_position(self):
        """返回夹爪闭合度，0 表示完全张开，1 表示完全闭合。"""
        return 1 - (self._gripper.get_state().width / self._max_gripper_width)

    def get_ee_pose(self):
        """获取末端执行器的笛卡尔位姿 (位置 + 欧拉角)。"""
        pos, quat = self._robot.get_ee_pose()
        angle = quat_to_euler(quat.numpy())
        return np.concatenate([pos, angle]).tolist()

    def get_robot_state(self):
        """读取机器人状态并附带时间戳信息。

        返回:
            tuple[dict, dict]: (状态字典, 时间戳字典)。
        """
        robot_state = self._robot.get_robot_state()
        gripper_position = self.get_gripper_position()
        pos, quat = self._robot.robot_model.forward_kinematics(torch.Tensor(robot_state.joint_positions))
        cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()

        state_dict = {
            "cartesian_position": cartesian_position,
            "gripper_position": gripper_position,
            "joint_positions": list(robot_state.joint_positions),
            "joint_velocities": list(robot_state.joint_velocities),
            "joint_torques_computed": list(robot_state.joint_torques_computed),
            "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(robot_state.motor_torques_measured),
            "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
            "prev_command_successful": robot_state.prev_command_successful,
        }

        timestamp_dict = {
            "robot_timestamp_seconds": robot_state.timestamp.seconds,
            "robot_timestamp_nanos": robot_state.timestamp.nanos,
        }

        return state_dict, timestamp_dict

    def adaptive_time_to_go(self, desired_joint_position, t_min=0, t_max=4):
        """估算从当前位置到目标关节位置的安全执行时长。

        参数:
            desired_joint_position (Tensor): 目标关节位置。
            t_min (float): 最小允许时间。
            t_max (float): 最大允许时间。

        返回:
            float: 裁剪后的执行时间。
        """
        curr_joint_position = self._robot.get_joint_positions()
        displacement = desired_joint_position - curr_joint_position
        time_to_go = self._robot._adaptive_time_to_go(displacement)
        clamped_time_to_go = min(t_max, max(time_to_go, t_min))
        return clamped_time_to_go

    def create_action_dict(self, action, action_space, gripper_action_space=None, robot_state=None):
        """根据动作空间解析动作向量，生成关节/笛卡尔及夹爪命令。

        参数:
            action (Sequence[float]): 动作向量，末尾元素始终对应夹爪控制。
            action_space (str): 机器人动作空间标识，支持笛卡尔/关节的速度或位置模式。
            gripper_action_space (str, 可选): 夹爪动作空间，缺省时依据机器人模式推断。
            robot_state (dict, 可选): 已知的机器人状态，缺省时内部调用 `get_robot_state` 获取。

        返回:
            dict: 包含关节/笛卡尔位姿、速度以及夹爪控制量的综合字典。
        """
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        if robot_state is None:
            robot_state = self.get_robot_state()[0]
        action_dict = {"robot_state": robot_state}
        velocity = "velocity" in action_space

        if gripper_action_space is None:
            gripper_action_space = "velocity" if velocity else "position"
        assert gripper_action_space in ["velocity", "position"]
            

        if gripper_action_space == "velocity":
            action_dict["gripper_velocity"] = action[-1]
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(action[-1])
            gripper_position = robot_state["gripper_position"] + gripper_delta
            action_dict["gripper_position"] = float(np.clip(gripper_position, 0, 1))
        else:
            action_dict["gripper_position"] = float(np.clip(action[-1], 0, 1))
            gripper_delta = action_dict["gripper_position"] - robot_state["gripper_position"]
            gripper_velocity = self._ik_solver.gripper_delta_to_velocity(gripper_delta)
            action_dict["gripper_delta"] = gripper_velocity

        if "cartesian" in action_space:
            if velocity:
                action_dict["cartesian_velocity"] = action[:-1]
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(action[:-1])
                action_dict["cartesian_position"] = add_poses(
                    cartesian_delta, robot_state["cartesian_position"]
                ).tolist()
            else:
                action_dict["cartesian_position"] = action[:-1]
                cartesian_delta = pose_diff(action[:-1], robot_state["cartesian_position"])
                cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
                action_dict["cartesian_velocity"] = cartesian_velocity.tolist()

            action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
                action_dict["cartesian_velocity"], robot_state=robot_state
            ).tolist()
            joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
            action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()

        if "joint" in action_space:
            # NOTE: Joint to Cartesian has undefined dynamics due to IK
            if velocity:
                action_dict["joint_velocity"] = action[:-1]
                joint_delta = self._ik_solver.joint_velocity_to_delta(action[:-1])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
            else:
                action_dict["joint_position"] = action[:-1]
                joint_delta = np.array(action[:-1]) - np.array(robot_state["joint_positions"])
                joint_velocity = self._ik_solver.joint_delta_to_velocity(joint_delta)
                action_dict["joint_velocity"] = joint_velocity.tolist()

        return action_dict
