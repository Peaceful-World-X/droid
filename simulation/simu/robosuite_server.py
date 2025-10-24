# -*- encoding: utf-8 -*-
'''
@File    :   robosuite_server.py
@Time    :   2025/09/28 22:24:42
@Author  :   Peaceful_World
@Version :   V2.0
@Contact :   Peaceful_World@qq.com
'''

"""基于 robosuite 的 Franka 仿真服务端"""

import sys
import threading
import time
from pathlib import Path

import numpy as np
import robosuite as suite
from robosuite.devices import Keyboard

simulation_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(simulation_dir))
from common import setup_logger


class RobosuiteServer:
    """基于 robosuite 的 Franka 仿真服务端"""

    def __init__(self):
        """初始化仿真环境."""
        self.logger = setup_logger("robosuite_r7arm", log_dir=simulation_dir / "log")

        self.env = None
        self.obs = None
        self.is_running = False
        self.sim_thread = None
        self.device = None  # Keyboard 设备

        # 统计信息
        self.step_count = 0  # 当前episode的步数

        # 初始化环境
        self._init_robosuite()

        # 初始化键盘设备
        self._init_keyboard_device()

    def _init_robosuite(self):
        """初始化 robosuite 环境."""
        self.logger.info("初始化 robosuite 环境...")

        # 创建环境 - 使用默认控制器（适用于 Keyboard 设备）
        self.env = suite.make(
            env_name='Lift',
            robots='Panda',
            gripper_types='Robotiq85Gripper',
            # 不指定控制器，使用默认的 OSC_POSE
            control_freq=30,
            has_renderer=True,
            render_camera=None,  # 自由相机模式
            has_offscreen_renderer=False,
            use_camera_obs=False,
            horizon=10000,  # 增加最大步数，避免频繁重置（默认约500步）
        )

        # 重置环境
        self.obs = self.env.reset()
        self.step_count = 0

        self.logger.info("robosuite 环境初始化完成")

    def _init_keyboard_device(self):
        """初始化键盘设备（使用 robosuite 的 Keyboard 类）."""
        self.device = Keyboard(env=self.env, pos_sensitivity=1.0, rot_sensitivity=1.0)
        self.device.start_control()
        self.logger.info("键盘设备已初始化")
        self.logger.info("按键说明：")
        self.logger.info("  上下左右: 水平移动 (x-y平面)")
        self.logger.info("  .-;: 垂直移动 (z轴)")
        self.logger.info("  o-p: 旋转 (yaw)")
        self.logger.info("  y-h: 旋转 (pitch)")
        self.logger.info("  e-r: 旋转 (roll)")
        self.logger.info("  空格: 切换夹爪开关")
        self.logger.info("  s: 切换活动手臂")
        self.logger.info("  =: 切换活动机器人")
        self.logger.info("  Ctrl+q: 重置仿真")

    def _simulation_loop(self):
        """仿真循环（使用 robosuite 的 Keyboard 设备）."""
        while self.is_running:
            try:
                # 从键盘设备获取状态
                state = self.device.get_controller_state()

                # 检查是否需要重置
                if state['reset']:
                    self.logger.warning("用户请求重置环境")
                    self.obs = self.env.reset()
                    self.step_count = 0
                    self.device.start_control()
                    continue

                # 构建动作：OSC_POSE 需要 [dpos(3), drotation(3), grasp(1)]
                # dpos: 位置增量 (x, y, z)
                # raw_drotation: 旋转增量 (roll, pitch, yaw)
                # grasp: 夹爪状态 (-1=关闭, 1=打开)
                action = np.concatenate([
                    state['dpos'],           # 3维位置增量
                    state['raw_drotation'],  # 3维旋转增量
                    [state['grasp']]         # 1维夹爪
                ])

                # 执行环境步进
                self.obs, reward, done, info = self.env.step(action)
                self.step_count += 1

                # 环境终止则重置
                if done:
                    reason = "未知原因"
                    if self.step_count >= 10000:
                        reason = f"达到最大步数限制 ({self.step_count} 步)"
                    elif "success" in info and info["success"]:
                        reason = f"任务成功完成 ({self.step_count} 步)"
                    else:
                        reason = f"其他原因 ({self.step_count} 步)"

                    self.logger.warning(f"环境已终止: {reason}")
                    self.obs = self.env.reset()
                    self.step_count = 0
                    self.device.start_control()

                # 渲染
                self.env.render()

            except Exception as e:
                self.logger.error(f"仿真循环出错: {e}")
                import traceback
                self.logger.error(traceback.format_exc())

            time.sleep(0.05)  # 20Hz

    def start(self):
        """启动仿真循环."""
        if self.is_running:
            self.logger.warning("仿真已在运行中")
            return

        self.is_running = True
        self.sim_thread = threading.Thread(target=self._simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        self.logger.info("仿真循环已启动")

    def stop(self):
        """停止仿真循环."""
        self.logger.info("正在停止...")

        # 设置停止标志
        self.is_running = False

        # 停止键盘监听器
        if self.device and hasattr(self.device, 'listener'):
            try:
                self.device.listener.stop()
            except Exception:
                pass

        # 等待仿真线程
        if self.sim_thread and self.sim_thread.is_alive():
            self.sim_thread.join(timeout=0.3)

        # 关闭环境
        if self.env:
            try:
                self.env.close()
                self.logger.info("环境已关闭")
            except Exception as e:
                self.logger.warning(f"关闭环境时出错: {e}")

        self.logger.info("仿真已停止")

    # ==================== 状态查询 ====================

    def get_joint_positions(self):
        """获取当前关节位置."""
        if self.obs and 'robot0_joint_pos' in self.obs:
            return self.obs['robot0_joint_pos'].copy()
        return None

    def get_ee_position(self):
        """获取当前末端位置."""
        if self.obs and 'robot0_eef_pos' in self.obs:
            return self.obs['robot0_eef_pos'].copy()
        return None

    def get_ee_orientation(self):
        """获取当前末端姿态（四元数）."""
        if self.obs and 'robot0_eef_quat' in self.obs:
            return self.obs['robot0_eef_quat'].copy()
        return None


def main():
    """主函数 - 示例."""
    logger = setup_logger("robosuite_main")
    logger.info("启动 Robosuite 仿真...")
    server = RobosuiteServer()

    try:
        server.start()

        # 保持运行
        while server.is_running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("\n收到中断信号，正在退出...")
    finally:
        server.stop()
        logger.info("程序结束")


if __name__ == "__main__":
    main()