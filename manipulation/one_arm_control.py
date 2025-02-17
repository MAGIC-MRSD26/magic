import mujoco as mj
from mujoco import viewer
import numpy as np
import math

class SingleArmControl():

    def __init__(self, model, data):

        self.desired_joint_ang = np.zeros(7)
        mj.mj_resetDataKeyframe(model, data, 0)
        self.home_joint_ang = data.qpos.copy()

    def PID_control_home(self, model, data):

        # Basic PD control
        Kp = 150.0 # P gain
        Kd = 0.8   # D gains
        
        gravity = data.qfrc_bias[:14]  # Gravity compensation terms
        position_error = data.qpos[:14] - self.home_joint_ang 
        velocity_damping = Kd * data.qvel[:14]
        
        data.ctrl[:14] = gravity - Kp * position_error - velocity_damping

    def go_to_home(self):

        # Use PID Control to go to home position
        mj.set_mjcb_control(self.PID_control_home)

    def single_arm_IK(self, goal):

        # Need to do this

        self.desired_joint_ang = np.zeros(7)



    # def impedance_control(self):

    # def open_gripper(self):

    # def close_gripper(self):