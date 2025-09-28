# demo_same_api.py
import argparse
import numpy as np
from sim_franky import SimFranky
# import real franky backend when running on robot:
# from franky import Robot as RealFranky

def demo(robot):
    print("连接后读取初始关节（q）:")
    q0 = robot.state()['q']
    print(q0)
    # 小幅度关节偏移（上层逻辑）—— 这个调用对仿真/真机相同
    q_target = q0.copy()
    q_target[0] += 0.3
    print("移动到 q_target（持续 2s）...")
    robot.move_joint_target(q_target, duration=2.0)
    print("回到起始位姿（持续 2s）...")
    robot.move_joint_target(q0, duration=2.0)
    # 笛卡尔相对位移
    print("笛卡尔相对移动 +0.1m Z（持续 1.5s）...")
    robot.move_cartesian_relative([0,0,0.1], duration=1.5)
    print("演示结束")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=['sim','real'], default='sim')
    args = parser.parse_args()
    if args.backend == 'sim':
        r = SimFranky(urdf_path='franka_panda.urdf', gui=True)
        r.connect()
        r.set_dynamic_rel(0.5)  # 在仿真中我们也可以用 same API 调整动态尺度
        try:
            demo(r)
        finally:
            r.disconnect()
    else:
        # 真机示例（伪码 — 真实情况需要 franky-control 包并按其 API）
        raise NotImplementedError("切换到真实 franky 后端：在这里实例化 franky 控件（franky-control/pip包）并调用相同的 demo(robot)")
