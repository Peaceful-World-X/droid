#!/usr/bin/env python3
"""Robosuite 服务端的测试客户端."""

import json
import socket
import time

import numpy as np


class RobosuiteClient:
    """Robosuite 服务端的测试客户端."""

    def __init__(self, host='localhost', port=8888):
        """初始化客户端."""
        self.host = host
        self.port = port
        self.socket = None

    def connect(self):
        """连接到服务端."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"已连接到服务端 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接."""
        if self.socket:
            self.socket.close()
            print("已断开连接")

    def send_command(self, command):
        """发送命令到服务端."""
        try:
            # 发送命令
            self.socket.send(json.dumps(command).encode('utf-8'))

            # 接收响应
            response = self.socket.recv(4096)
            return json.loads(response.decode('utf-8'))
        except Exception as e:
            print(f"发送命令失败: {e}")
            return None

    def get_status(self):
        """获取机器人状态."""
        command = {"type": "get_status"}
        return self.send_command(command)

    def pause_simulation(self):
        """暂停仿真."""
        command = {"type": "pause"}
        return self.send_command(command)

    def resume_simulation(self):
        """恢复仿真."""
        command = {"type": "resume"}
        return self.send_command(command)

    def stop_server(self):
        """停止服务端."""
        command = {"type": "stop"}
        return self.send_command(command)

    def set_joint_position(self, positions):
        """设置关节位置."""
        command = {
            "type": "joint_position",
            "positions": positions
        }
        return self.send_command(command)

    def set_joint_velocity(self, velocities):
        """设置关节速度 [v1, v2, v3, v4, v5, v6, v7]."""
        command = {
            "type": "joint_velocity",
            "velocities": velocities
        }
        return self.send_command(command)

    def set_ee_position(self, position):
        """设置末端执行器位置 [x, y, z]."""
        command = {
            "type": "ee_position",
            "position": position
        }
        return self.send_command(command)

    def set_ee_velocity(self, velocity):
        """设置末端执行器速度 [vx, vy, vz, wx, wy, wz]."""
        command = {
            "type": "ee_velocity",
            "velocity": velocity
        }
        return self.send_command(command)


def demo_control_modes():
    """演示各种控制模式."""
    client = RobosuiteClient()

    if not client.connect():
        return

    try:
        print("=" * 50)
        print("Robosuite Franka 仿真客户端演示")
        print("=" * 50)

        # 获取初始状态
        print("\n1. 获取初始状态")
        status = client.get_status()
        if status and status.get('status') == 'success':
            data = status['data']
            print(f"当前控制模式: {data['control_mode']}")
            print(f"是否暂停: {data['is_paused']}")
            print(f"当前关节位置: {np.array(data['current_joint_pos'])}")
            print(f"当前末端位置: {np.array(data['current_ee_position'])}")
            print(f"当前末端姿态: {np.array(data['current_ee_orientation'])}")

            # 保存初始位置
            initial_joint_pos = data['current_joint_pos']
            initial_ee_pos = data['current_ee_position']
            initial_ee_orient = data['current_ee_orientation']

        input("\n按回车键继续...")

        # 测试关节位置控制
        print("\n2. 测试关节位置控制")
        # 创建一个简单的关节位置目标
        target_joints = [0.1, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8]
        response = client.set_joint_position(target_joints)
        print(f"设置关节位置: {response}")

        time.sleep(3)  # 等待机械臂移动

        input("\n按回车键继续...")

        # 测试末端位置控制
        print("\n3. 测试末端位置控制")
        # 在当前位置基础上移动
        if initial_ee_pos:
            target_pos = [
                initial_ee_pos[0] + 0.1,  # x + 10cm
                initial_ee_pos[1] + 0.1,  # y + 10cm
                initial_ee_pos[2] + 0.1   # z + 10cm
            ]
            response = client.set_ee_position(target_pos)
            print(f"设置末端位置: {response}")

        time.sleep(3)

        input("\n按回车键继续...")

        # 测试末端位姿控制
        print("\n4. 测试末端位姿控制")
        if initial_ee_pos and initial_ee_orient:
            target_pose = [
                initial_ee_pos[0],      # x
                initial_ee_pos[1],      # y
                initial_ee_pos[2] + 0.2, # z + 20cm
                initial_ee_orient[0],   # rx
                initial_ee_orient[1],   # ry
                initial_ee_orient[2] + 0.5  # rz + 0.5 rad
            ]
            response = client.set_ee_pose(target_pose)
            print(f"设置末端位姿: {response}")

        time.sleep(3)

        input("\n按回车键继续...")

        # 测试末端速度控制
        print("\n5. 测试末端速度控制")
        # 设置一个缓慢的直线运动
        velocity = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]  # 只在x方向移动
        response = client.set_ee_velocity(velocity)
        print(f"设置末端速度: {response}")

        time.sleep(2)

        # 停止速度控制
        zero_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        client.set_ee_velocity(zero_velocity)

        input("\n按回车键继续...")

        # 测试暂停/恢复
        print("\n6. 测试暂停/恢复功能")
        response = client.pause_simulation()
        print(f"暂停仿真: {response}")

        time.sleep(2)

        response = client.resume_simulation()
        print(f"恢复仿真: {response}")

        input("\n按回车键继续...")

        # 返回初始位置
        print("\n7. 返回初始位置")
        if initial_joint_pos:
            response = client.set_joint_position(initial_joint_pos)
            print(f"返回初始关节位置: {response}")

        time.sleep(3)

        # 最终状态
        print("\n8. 最终状态")
        status = client.get_status()
        if status and status.get('status') == 'success':
            data = status['data']
            print(f"最终关节位置: {np.array(data['current_joint_pos'])}")
            print(f"最终末端位置: {np.array(data['current_ee_position'])}")

        print("\n演示完成！")

    except KeyboardInterrupt:
        print("\n演示被用户中断")
    except Exception as e:
        print(f"演示过程中出错: {e}")
    finally:
        client.disconnect()


def interactive_mode():
    """交互模式."""
    client = RobosuiteClient()

    if not client.connect():
        return

    try:
        print("\n进入交互模式. 输入 'help' 查看帮助, 输入 'quit' 退出")

        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == 'quit':
                break
            elif cmd == 'help':
                print_help()
            elif cmd == 'status':
                response = client.get_status()
                print_response(response)
            elif cmd == 'pause':
                response = client.pause_simulation()
                print_response(response)
            elif cmd == 'resume':
                response = client.resume_simulation()
                print_response(response)
            elif cmd.startswith('joint '):
                try:
                    parts = cmd.split()[1:]
                    if len(parts) == 7:
                        positions = [float(x) for x in parts]
                        response = client.set_joint_position(positions)
                        print_response(response)
                    else:
                        print("需要 7 个关节位置值")
                except:
                    print("关节位置格式错误")
            elif cmd.startswith('jointvel'):
                try:
                    parts = cmd.split()[1:]
                    if len(parts) == 7:
                        velocities = [float(x) for x in parts]
                        response = client.set_joint_velocity(velocities)
                        print_response(response)
                    else:
                        print("需要 7 个关节速度值")
                except:
                    print("关节速度格式错误")
            elif cmd.startswith('pos'):
                try:
                    parts = cmd.split()[1:]
                    if len(parts) == 3:
                        position = [float(x) for x in parts]
                        response = client.set_ee_position(position)
                        print_response(response)
                    else:
                        print("需要 3 个位置值 [x, y, z]")
                except:
                    print("位置格式错误")
            elif cmd.startswith('vel'):
                try:
                    parts = cmd.split()[1:]
                    if len(parts) == 6:
                        velocity = [float(x) for x in parts]
                        response = client.set_ee_velocity(velocity)
                        print_response(response)
                    else:
                        print("需要 6 个速度值 [vx, vy, vz, wx, wy, wz]")
                except:
                    print("速度格式错误")
            else:
                print("未知命令，输入 'help' 查看帮助")

    except KeyboardInterrupt:
        print("\n交互模式被用户中断")
    except Exception as e:
        print(f"交互模式出错: {e}")
    finally:
        client.disconnect()


def print_help():
    """打印帮助信息."""
    help_text = """
可用命令:
  help                                    - 显示此帮助
  status                                  - 获取机器人状态
  pause                                   - 暂停仿真
  resume                                  - 恢复仿真
  joint <j1> <j2> <j3> <j4> <j5> <j6> <j7> - 设置关节位置
  jointvel <v1> <v2> <v3> <v4> <v5> <v6> <v7> - 设置关节速度
  pos <x> <y> <z>                         - 设置末端位置
  vel <vx> <vy> <vz> <wx> <wy> <wz>       - 设置末端速度
  quit                                    - 退出

示例:
  joint 0.1 -0.5 0.0 -2.0 0.0 1.5 0.8
  jointvel 0.5 0.0 0.0 0.0 0.0 0.0 0.0
  pos 0.5 0.2 0.4
  vel 0.1 0.0 0.0 0.0 0.0 0.0
    """
    print(help_text)


def print_response(response):
    """打印服务端响应."""
    if response:
        if response.get('status') == 'success':
            print(f"✓ {response.get('message', 'Success')}")
            if 'data' in response:
                data = response['data']
                print(f"  控制模式: {data.get('control_mode', 'N/A')}")
                if 'current_joint_pos' in data:
                    joints = np.array(data['current_joint_pos'])
                    print(f"  关节位置: {joints}")
                if 'current_ee_position' in data:
                    pos = np.array(data['current_ee_position'])
                    print(f"  末端位置: {pos}")
        else:
            print(f"✗ {response.get('message', 'Error')}")
    else:
        print("✗ 无响应")


def demo_single_axis_movement():
    """演示单轴移动控制."""
    client = RobosuiteClient()

    if not client.connect():
        return

    try:
        print("=" * 60)
        print("Robosuite Franka 单轴移动控制演示")
        print("=" * 60)

        # 获取初始状态
        print("\n获取初始状态...")
        status = client.get_status()
        if not status or status.get('status') != 'success':
            print("无法获取机器人状态")
            return

        data = status['data']
        initial_joint_pos = np.array(data['current_joint_pos'])
        initial_ee_pos = np.array(data['current_ee_position'])
        initial_ee_orient = np.array(data['current_ee_orientation'])

        print(f"初始关节位置: {initial_joint_pos}")
        print(f"初始末端位置: {initial_ee_pos}")
        print(f"初始末端姿态: {initial_ee_orient}")

        # 定义移动步长
        joint_step = 0.3  # 关节角度步长 (弧度)
        position_step = 0.15  # 位置步长 (米)
        orientation_step = 0.5  # 姿态步长 (弧度)

        input("\n按回车键开始演示...")

        # ==================== 关节位置控制演示 ====================
        print("\n" + "="*50)
        print("1. 关节位置控制 - 单关节移动演示")
        print("="*50)

        # 演示每个关节的单独移动
        joint_names = ["肩部转动", "肩部俯仰", "上臂翻转", "肘部弯曲", "前臂翻转", "腕部俯仰", "腕部翻转"]

        for i in range(7):
            print(f"\n移动第{i+1}个关节 ({joint_names[i]})...")

            # 正向移动
            target_joints = initial_joint_pos.copy()
            target_joints[i] += joint_step
            response = client.set_joint_position(target_joints.tolist())
            print(f"  正向移动: {response.get('message', 'Error')}")
            time.sleep(2)

            # 负向移动
            target_joints[i] = initial_joint_pos[i] - joint_step
            response = client.set_joint_position(target_joints.tolist())
            print(f"  负向移动: {response.get('message', 'Error')}")
            time.sleep(2)

            # 回到初始位置
            response = client.set_joint_position(initial_joint_pos.tolist())
            print(f"  回到初始位置: {response.get('message', 'Error')}")
            time.sleep(1.5)

            if i < 6:  # 不是最后一个关节
                input(f"第{i+1}个关节演示完成，按回车键继续下一个关节...")

        print("关节位置控制演示完成！")
        input("\n按回车键继续关节速度控制演示...")

        # ==================== 关节速度控制演示 ====================
        print("\n" + "="*50)
        print("2. 关节速度控制 - 单关节速度演示")
        print("="*50)

        # 确保回到初始位置
        client.set_joint_position(initial_joint_pos.tolist())
        time.sleep(2)

        # 演示每个关节的速度控制
        velocity_step = 0.5  # 关节速度步长 (弧度/秒)

        for i in range(7):
            print(f"\n控制第{i+1}个关节速度 ({joint_names[i]})...")

            # 正向速度
            target_velocities = np.zeros(7)
            target_velocities[i] = velocity_step
            response = client.set_joint_velocity(target_velocities.tolist())
            print(f"  正向速度: {response.get('message', 'Error')}")
            time.sleep(1.5)

            # 停止运动
            zero_velocities = np.zeros(7)
            response = client.set_joint_velocity(zero_velocities.tolist())
            print(f"  停止运动: {response.get('message', 'Error')}")
            time.sleep(0.5)

            # 负向速度
            target_velocities[i] = -velocity_step
            response = client.set_joint_velocity(target_velocities.tolist())
            print(f"  负向速度: {response.get('message', 'Error')}")
            time.sleep(1.5)

            # 停止运动
            response = client.set_joint_velocity(zero_velocities.tolist())
            print(f"  停止运动: {response.get('message', 'Error')}")
            time.sleep(0.5)

            if i < 6:  # 不是最后一个关节
                input(f"第{i+1}个关节速度演示完成，按回车键继续下一个关节...")

        print("关节速度控制演示完成！")
        input("\n按回车键继续末端位置控制演示...")

        # ==================== 末端位置控制演示 ====================
        print("\n" + "="*50)
        print("3. 末端位置控制 - 单轴移动演示")
        print("="*50)

        # 确保回到初始位置
        client.set_joint_position(initial_joint_pos.tolist())
        time.sleep(2)

        # 演示末端位置的 XYZ 轴移动
        axis_names = ["X轴 (前后)", "Y轴 (左右)", "Z轴 (上下)"]

        for i in range(3):
            print(f"\n移动末端位置 - {axis_names[i]}...")

            # 正向移动
            target_position = initial_ee_pos.copy()
            target_position[i] += position_step
            response = client.set_ee_position(target_position.tolist())
            print(f"  {axis_names[i]} 正向移动: {response.get('message', 'Error')}")
            time.sleep(2)

            # 负向移动
            target_position[i] = initial_ee_pos[i] - position_step
            response = client.set_ee_position(target_position.tolist())
            print(f"  {axis_names[i]} 负向移动: {response.get('message', 'Error')}")
            time.sleep(2)

            # 回到初始位置
            response = client.set_ee_position(initial_ee_pos.tolist())
            print(f"  回到初始位置: {response.get('message', 'Error')}")
            time.sleep(1.5)

            if i < 2:  # 不是最后一个轴
                input(f"{axis_names[i]} 位置演示完成，按回车键继续下一个轴...")

        print("末端位置控制演示完成！")
        input("\n按回车键继续末端速度控制演示...")

        # ==================== 末端速度控制演示 ====================
        print("\n" + "="*50)
        print("4. 末端速度控制 - 单轴速度演示")
        print("="*50)

        # 确保回到初始位置
        client.set_joint_position(initial_joint_pos.tolist())
        time.sleep(2)

        # 演示末端线速度控制
        linear_velocity_step = 0.05  # 线速度步长 (米/秒)
        angular_velocity_step = 0.3  # 角速度步长 (弧度/秒)

        # 线速度演示
        velocity_names = ["X方向线速度", "Y方向线速度", "Z方向线速度", "X轴角速度", "Y轴角速度", "Z轴角速度"]

        for i in range(6):
            print(f"\n控制末端 - {velocity_names[i]}...")

            # 正向速度
            target_velocity = np.zeros(6)
            if i < 3:  # 线速度
                target_velocity[i] = linear_velocity_step
            else:  # 角速度
                target_velocity[i] = angular_velocity_step

            response = client.set_ee_velocity(target_velocity.tolist())
            print(f"  正向速度: {response.get('message', 'Error')}")
            time.sleep(1.5)

            # 停止运动
            zero_velocity = np.zeros(6)
            response = client.set_ee_velocity(zero_velocity.tolist())
            print(f"  停止运动: {response.get('message', 'Error')}")
            time.sleep(0.5)

            # 负向速度
            if i < 3:  # 线速度
                target_velocity[i] = -linear_velocity_step
            else:  # 角速度
                target_velocity[i] = -angular_velocity_step

            response = client.set_ee_velocity(target_velocity.tolist())
            print(f"  负向速度: {response.get('message', 'Error')}")
            time.sleep(1.5)

            # 停止运动
            response = client.set_ee_velocity(zero_velocity.tolist())
            print(f"  停止运动: {response.get('message', 'Error')}")
            time.sleep(0.5)

            if i < 5:  # 不是最后一个轴
                input(f"{velocity_names[i]} 演示完成，按回车键继续下一个轴...")

        print("\n" + "="*50)
        print("单轴移动控制演示全部完成！")
        print("="*50)

        # 最终回到初始位置
        print("\n回到初始关节位置...")
        client.set_joint_position(initial_joint_pos.tolist())
        time.sleep(2)

        # 显示最终状态
        final_status = client.get_status()
        if final_status and final_status.get('status') == 'success':
            final_data = final_status['data']
            print(f"最终关节位置: {np.array(final_data['current_joint_pos'])}")
            print(f"最终末端位置: {np.array(final_data['current_ee_position'])}")
            print(f"最终末端姿态: {np.array(final_data['current_ee_orientation'])}")

    except KeyboardInterrupt:
        print("\n演示被用户中断")
    except Exception as e:
        print(f"演示过程中出错: {e}")
    finally:
        client.disconnect()


def main():
    """主函数."""
    print("Robosuite Franka 仿真客户端")
    print("请确保服务端已启动 (运行 robosuite_server.py)")

    while True:
        print("\n选择模式:")
        print("1. 综合演示模式 - 自动演示各种控制功能")
        print("2. 单轴演示模式 - 演示单个方向的精确移动")
        print("3. 交互模式 - 手动输入控制命令")
        print("4. 退出")

        choice = input("请选择 (1-4): ").strip()

        if choice == '1':
            demo_control_modes()
        elif choice == '2':
            demo_single_axis_movement()
        elif choice == '3':
            interactive_mode()
        elif choice == '4':
            break
        else:
            print("无效选择，请重试")


if __name__ == "__main__":
    main()