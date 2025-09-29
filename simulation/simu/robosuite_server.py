# -*- encoding: utf-8 -*-
'''
@File    :   robosuite_server.py
@Time    :   2025/09/28 22:24:42
@Author  :   Peaceful_World
@Version :   V1.0
@Contact :   Peaceful_World@qq.com
'''

"""基于 robosuite 的 Franka[r7arm] 仿真服务端."""

import json
import os
import socket
import sys
import threading
import time

import numpy as np
import yaml

sys.path.insert(0, '/home/joker/Code_Work/Code/robosuite')
import robosuite as suite
from robosuite.utils.transform_utils import euler2mat, mat2euler, mat2quat, quat2mat


class RobosuiteServer:
    """基于 robosuite 的 Franka 仿真服务端."""

    def __init__(self, config_file='config.yaml'):
        """初始化服务端."""
        # 加载配置文件
        self.config = self._load_config(config_file)

        # 服务端配置
        self.host = self.config['server']['host']
        self.port = self.config['server']['port']
        self.socket = None
        self.env = None
        self.obs = None

        # 控制状态
        self.is_paused = False
        self.is_running = True
        self.control_mode = self.config['control']['default_mode']

        # 目标位置/姿态
        self.target_joint_pos = None
        self.target_joint_vel = None
        self.target_ee_position = None
        self.target_ee_velocity = None

        # 初始化 robosuite 环境
        self._init_robosuite()

    def _load_config(self, config_file):
        """加载配置文件."""
        config_path = os.path.join(os.path.dirname(__file__), config_file)
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            print(f"成功加载配置文件: {config_path}")
            return config
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            return self._get_default_config()

    def _get_default_config(self):
        """获取默认配置."""
        return {
            'server': {'host': 'localhost', 'port': 8888},
            'environment': {
                'env_name': 'Lift',
                'robots': 'Panda',
                'gripper_types': 'Robotiq85Gripper',
                'control_freq': 30,
                'has_renderer': True,
                'render_camera': 'sideview',
                'has_offscreen_renderer': False,
                'use_camera_obs': False
            },
            'camera': {
                'names': ['frontview', 'sideview', 'robot0_robotview', 'robot0_eye_in_hand'],
                'heights': 480,
                'widths': 640,
                'depths': False,
            },
            'control': {
                'simulation_freq': 20,
                'default_mode': 'joint_position',
                'action_limits': {
                    'joint_position': 0.1,
                    'joint_velocity': 0.2,
                    'ee_position': 0.1,
                    'ee_velocity': 0.2
                }
            }
        }

    def _init_robosuite(self):
        """初始化 robosuite 环境."""
        print("初始化 robosuite 环境...")

        # 从配置获取控制器设置
        env_config = self.config['environment']
        cam_config = self.config['camera']

        # 创建环境（使用默认的 JOINT_POSITION 控制器，返回 7 维动作）
        self.env = suite.make(
            env_name=env_config['env_name'],
            robots=env_config['robots'],
            gripper_types=env_config['gripper_types'],
            control_freq=env_config['control_freq'],
            has_renderer=env_config['has_renderer'],
            render_camera=env_config['render_camera'],
            has_offscreen_renderer=env_config['has_offscreen_renderer'],
            camera_names=cam_config['names'],
            camera_heights=cam_config['heights'],
            camera_widths=cam_config['widths'],
            camera_depths=cam_config['depths'],
            use_camera_obs=env_config['use_camera_obs'],
            # 不指定 controller_configs，使用默认控制器
        )

        # 重置环境
        self.obs = self.env.reset()
        print("robosuite 环境初始化完成")

        # 获取初始关节位置和速度
        self.target_joint_pos = self.obs['robot0_joint_pos'].copy()
        self.target_joint_vel = np.zeros(7)  # 初始关节速度为零

        # 获取初始末端执行器位置
        ee_pos = self.obs['robot0_eef_pos']
        self.target_ee_position = ee_pos.copy()
        self.target_ee_velocity = np.zeros(6)  # 6维速度：线速度+角速度

    def start_server(self):
        """启动服务端."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            print(f"服务端启动，监听 {self.host}:{self.port}")

            # 启动仿真循环线程
            sim_thread = threading.Thread(target=self._simulation_loop)
            sim_thread.daemon = True
            sim_thread.start()

            while self.is_running:
                try:
                    client_socket, addr = self.socket.accept()
                    print(f"客户端连接: {addr}")

                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_socket,)
                    )
                    client_thread.daemon = True
                    client_thread.start()

                except Exception as e:
                    print(f"接受连接时出错: {e}")

        except Exception as e:
            print(f"服务端启动失败: {e}")
        finally:
            self._cleanup()

    def _simulation_loop(self):
        """仿真循环."""
        while self.is_running:
            if not self.is_paused:
                try:
                    # 根据当前控制模式生成动作
                    action = self._generate_action()

                    # 执行动作
                    self.obs, reward, done, info = self.env.step(action)

                    # 渲染
                    self.env.render()

                except Exception as e:
                    print(f"仿真循环出错: {e}")

            time.sleep(1.0 / self.config['control']['simulation_freq'])  # 使用配置的仿真频率

    def _generate_action(self):
        """根据当前控制模式生成动作."""
        if self.control_mode == "joint_position":
            return self._joint_position_control()
        elif self.control_mode == "joint_velocity":
            return self._joint_velocity_control()
        elif self.control_mode == "ee_position":
            return self._ee_position_control()
        elif self.control_mode == "ee_velocity":
            return self._ee_velocity_control()
        else:
            return np.zeros(7)  # 默认 7 维动作（关节控制）

    def _joint_position_control(self):
        """关节位置控制."""
        if self.target_joint_pos is not None:
            current_joint_pos = self.obs['robot0_joint_pos']

            # 简单比例控制
            kp = 0.5
            action = kp * (self.target_joint_pos - current_joint_pos)

            # 使用配置的动作限制
            limit = self.config['control']['action_limits']['joint_position']
            action = np.clip(action, -limit, limit)
            return action
        return np.zeros(7)

    def _joint_velocity_control(self):
        """关节速度控制."""
        if self.target_joint_vel is not None:
            # 直接使用目标关节速度
            action = self.target_joint_vel.copy()

            # 使用配置的动作限制
            limit = self.config['control']['action_limits']['joint_velocity']
            action = np.clip(action, -limit, limit)
            return action
        return np.zeros(7)

    def _ee_position_control(self):
        """末端位置控制（通过简化的逆运动学转换为关节动作）."""
        if self.target_ee_position is not None:
            current_ee_pos = self.obs['robot0_eef_pos']
            current_joint_pos = self.obs['robot0_joint_pos']

            # 计算位置误差
            pos_error = self.target_ee_position - current_ee_pos

            # 简化的逆运动学：将位置误差映射到关节空间
            kp = 0.1
            joint_adjustment = np.zeros(7)

            # 将 XYZ 位置误差分配到前几个关节
            joint_adjustment[0] = kp * pos_error[0]  # 基座旋转影响 X
            joint_adjustment[1] = kp * pos_error[2]  # 肩部俯仰影响 Z
            joint_adjustment[2] = kp * pos_error[1]  # 上臂翻转影响 Y
            joint_adjustment[3] = kp * pos_error[2]  # 肘部弯曲影响 Z

            # 使用配置的动作限制
            limit = self.config['control']['action_limits']['ee_position']
            joint_adjustment = np.clip(joint_adjustment, -limit, limit)
            return joint_adjustment
        return np.zeros(7)

    def _ee_velocity_control(self):
        """末端速度控制（通过简化的雅可比矩阵转换为关节速度）."""
        if self.target_ee_velocity is not None:
            # 简化的雅可比矩阵：将末端速度映射到关节速度
            # 前3个是线速度，后3个是角速度
            linear_vel = self.target_ee_velocity[:3]
            angular_vel = self.target_ee_velocity[3:6]

            kp = 0.2
            joint_velocities = np.zeros(7)

            # 将线速度分配到前几个关节
            joint_velocities[0] = kp * linear_vel[0]  # X 方向
            joint_velocities[1] = kp * linear_vel[2]  # Z 方向
            joint_velocities[2] = kp * linear_vel[1]  # Y 方向
            joint_velocities[3] = kp * linear_vel[2]  # Z 方向

            # 将角速度分配到后几个关节
            joint_velocities[4] = kp * angular_vel[0]  # Roll
            joint_velocities[5] = kp * angular_vel[1]  # Pitch
            joint_velocities[6] = kp * angular_vel[2]  # Yaw

            # 使用配置的动作限制
            limit = self.config['control']['action_limits']['ee_velocity']
            joint_velocities = np.clip(joint_velocities, -limit, limit)
            return joint_velocities
        return np.zeros(7)

    def _handle_client(self, client_socket):
        """处理客户端请求."""
        try:
            while self.is_running:
                data = client_socket.recv(1024)
                if not data:
                    break

                try:
                    # 解析 JSON 命令
                    command = json.loads(data.decode('utf-8'))
                    response = self._process_command(command)

                    # 发送响应
                    client_socket.send(json.dumps(response).encode('utf-8'))

                except json.JSONDecodeError:
                    error_response = {"status": "error", "message": "Invalid JSON format"}
                    client_socket.send(json.dumps(error_response).encode('utf-8'))
                except Exception as e:
                    error_response = {"status": "error", "message": str(e)}
                    client_socket.send(json.dumps(error_response).encode('utf-8'))

        except Exception as e:
            print(f"处理客户端时出错: {e}")
        finally:
            client_socket.close()

    def _process_command(self, command):
        """处理客户端命令."""
        cmd_type = command.get('type')

        if cmd_type == 'pause':
            return self._handle_pause()
        elif cmd_type == 'resume':
            return self._handle_resume()
        elif cmd_type == 'stop':
            return self._handle_stop()
        elif cmd_type == 'joint_position':
            return self._handle_joint_position(command)
        elif cmd_type == 'joint_velocity':
            return self._handle_joint_velocity(command)
        elif cmd_type == 'ee_position':
            return self._handle_ee_position(command)
        elif cmd_type == 'ee_velocity':
            return self._handle_ee_velocity(command)
        elif cmd_type == 'get_status':
            return self._handle_get_status()
        else:
            return {"status": "error", "message": f"Unknown command type: {cmd_type}"}

    def _handle_pause(self):
        """处理暂停命令."""
        self.is_paused = True
        return {"status": "success", "message": "Simulation paused"}

    def _handle_resume(self):
        """处理恢复命令."""
        self.is_paused = False
        return {"status": "success", "message": "Simulation resumed"}

    def _handle_stop(self):
        """处理停止命令."""
        self.is_running = False
        return {"status": "success", "message": "Server stopping"}

    def _handle_joint_position(self, command):
        """处理关节位置控制命令."""
        try:
            positions = command.get('positions')
            if positions is None or len(positions) != 7:
                return {"status": "error", "message": "Invalid joint positions"}

            self.control_mode = "joint_position"
            self.target_joint_pos = np.array(positions)

            return {
                "status": "success",
                "message": "Joint position target set",
                "current_mode": self.control_mode
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _handle_joint_velocity(self, command):
        """处理关节速度控制命令."""
        try:
            velocities = command.get('velocities')
            if velocities is None or len(velocities) != 7:
                return {"status": "error", "message": "Invalid joint velocities"}

            self.control_mode = "joint_velocity"
            self.target_joint_vel = np.array(velocities)

            return {
                "status": "success",
                "message": "Joint velocity target set",
                "current_mode": self.control_mode
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _handle_ee_position(self, command):
        """处理末端位置控制命令."""
        try:
            position = command.get('position')
            if position is None or len(position) != 3:
                return {"status": "error", "message": "Invalid end-effector position (need [x,y,z])"}

            self.control_mode = "ee_position"
            self.target_ee_position = np.array(position)

            return {
                "status": "success",
                "message": "End-effector position target set",
                "current_mode": self.control_mode
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _handle_ee_velocity(self, command):
        """处理末端速度控制命令."""
        try:
            velocity = command.get('velocity')
            if velocity is None or len(velocity) != 6:
                return {"status": "error", "message": "Invalid end-effector velocity (need [vx,vy,vz,wx,wy,wz])"}

            self.control_mode = "ee_velocity"
            self.target_ee_velocity = np.array(velocity)

            return {
                "status": "success",
                "message": "End-effector velocity target set",
                "current_mode": self.control_mode
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _handle_get_status(self):
        """处理状态查询命令."""
        try:
            current_joint_pos = self.obs['robot0_joint_pos'].tolist()
            current_ee_pos = self.obs['robot0_eef_pos'].tolist()
            current_ee_quat = self.obs['robot0_eef_quat'].tolist()

            # 转换四元数为欧拉角
            ee_rot_mat = quat2mat(self.obs['robot0_eef_quat'])
            current_ee_euler = mat2euler(ee_rot_mat).tolist()

            return {
                "status": "success",
                "data": {
                    "is_paused": self.is_paused,
                    "control_mode": self.control_mode,
                    "current_joint_pos": current_joint_pos,
                    "current_ee_position": current_ee_pos,
                    "current_ee_orientation": current_ee_euler,
                    "current_ee_quaternion": current_ee_quat
                }
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _cleanup(self):
        """清理资源."""
        print("清理资源...")
        if self.socket:
            self.socket.close()
        if self.env:
            self.env.close()
        print("服务端已关闭")


def main():
    """主函数."""
    print("启动 Robosuite Franka 仿真服务端...")
    server = RobosuiteServer()  # 使用默认配置文件

    try:
        server.start_server()
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭服务端...")
        server.is_running = False
    except Exception as e:
        print(f"服务端运行出错: {e}")


if __name__ == "__main__":
    main()