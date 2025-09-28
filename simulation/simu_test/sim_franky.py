# sim_franky.py
import pybullet as p
import pybullet_data
import time
import numpy as np
from robot_interface import FrankyLike

class SimFranky(FrankyLike):
    def __init__(self, urdf_path='franka_panda.urdf', gui=True, time_step=1/240.0):
        self.urdf_path = urdf_path
        self.gui = gui
        self.time_step = time_step
        self.client = None
        self.robot_id = None
        self.joint_indices = None
        self.dynamics_scale = 1.0

    def connect(self):
        if self.gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        self.robot_id = p.loadURDF(self.urdf_path, useFixedBase=True)
        # pick revolute joints
        self.joint_indices = [i for i in range(p.getNumJoints(self.robot_id))
                              if p.getJointInfo(self.robot_id, i)[2] == p.JOINT_REVOLUTE]
        # disable default motors to use POSITION_CONTROL
        for ji in self.joint_indices:
            p.setJointMotorControl2(self.robot_id, ji, p.POSITION_CONTROL, targetPosition=0, force=0)

    def disconnect(self):
        if self.client is not None:
            p.disconnect(self.client)
            self.client = None

    def set_dynamic_rel(self, factor: float):
        # scale factor for speeds when using move commands
        self.dynamics_scale = float(max(1e-3, min(1.0, factor)))

    def get_joint_positions(self):
        return np.array([p.getJointState(self.robot_id, ji)[0] for ji in self.joint_indices])

    def move_joint_target(self, q_target, duration: float):
        q_target = np.asarray(q_target)
        cur = self.get_joint_positions()
        steps = max(2, int(duration / self.time_step))
        # scale speeds by dynamics_scale
        for i in range(1, steps+1):
            alpha = float(i) / steps
            q = cur + (q_target - cur) * alpha
            for ji, qv in zip(self.joint_indices, q):
                p.setJointMotorControl2(self.robot_id, ji, p.POSITION_CONTROL, targetPosition=float(qv), force=200)
            p.stepSimulation()
            time.sleep(self.time_step * (1.0 / self.dynamics_scale))
        return

    def move_cartesian_relative(self, pos_delta, duration: float):
        # naive: compute ik target and call move_joint_target
        end_eff = self.joint_indices[-1]
        link_state = p.getLinkState(self.robot_id, end_eff)
        cur_pos = np.array(link_state[4])  # world pos
        target_pos = cur_pos + np.asarray(pos_delta)
        current_q = self.get_joint_positions()
        q_ik = p.calculateInverseKinematics(self.robot_id, end_eff, target_pos)
        q_ik = np.array(q_ik[:len(self.joint_indices)])
        self.move_joint_target(q_ik, duration)

    def state(self):
        return {"q": self.get_joint_positions()}

    def stop(self):
        # quick stop: set zero velocity target
        for ji in self.joint_indices:
            p.setJointMotorControl2(self.robot_id, ji, p.VELOCITY_CONTROL, targetVelocity=0)
