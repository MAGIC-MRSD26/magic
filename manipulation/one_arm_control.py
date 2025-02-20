import mujoco as mj
from mujoco import viewer
import numpy as np
import math

class SingleArmControl():

    def __init__(self, model, data):

        self.num_actuators = model.nu
        self.desired_joint_ang = np.zeros(model.nu)

        self.closed_gripper_angle = 0.725
        self.open_gripper_angle = 0.0

        self.indexing_from_qpos = {
            "joint_1" : 0,
            "joint_2" : 1,
            "joint_3" : 2,
            "joint_4" : 3,
            "joint_5" : 4,
            "joint_6" : 5,
            "joint_7" : 6,
            "joint_1_2" : 13,
            "joint_2_2" : 14,
            "joint_3_2" : 15,
            "joint_4_2" : 16,
            "joint_5_2" : 17,
            "joint_6_2" : 18,
            "joint_7_2" : 19,
            "finger_joint" : 7,
            "finger_joint_2" : 20
        }

    def PID_control(self, model, data):

        # Basic PD control
        Kp = 100.0 # P gain for arm joint
        Kd = 2   # D gains for arm joint

        # Compute control for each actuator
        for i, (joint_name, index) in enumerate(self.indexing_from_qpos.items()):
            position_error = data.qpos[index] - self.desired_joint_ang[i]
            
            # Choose gains based on actuator type
            if 'finger' in joint_name:
                data.ctrl[i] = self.desired_joint_ang[i]
            else:
                velocity_damping = Kd * data.qvel[index]
                data.ctrl[i] = data.qfrc_bias[index] - Kp * position_error - velocity_damping

    def go_to_home(self, model, data):

        # Reset to home position keyframe
        mj.mj_resetDataKeyframe(model, data, 0)
        for i, (joint_name, index) in enumerate(self.indexing_from_qpos.items()):
            self.desired_joint_ang[i] = data.qpos[index].copy()

        mj.set_mjcb_control(self.PID_control)

    def single_arm_IK(self, goal):

        # Need to do this

        self.desired_joint_ang = np.zeros(7)

    def set_arm_target(self, arm_number, joint_angles):
        """
        Set target angles for a specific arm (1 or 2)
        joint_angles should be list of 7 angles for the arm joints
        """
        if len(joint_angles) != 7:
            raise ValueError("Must provide 7 joint angles")
            
        # Set joint angles for the specified arm
        start_idx = 0 if arm_number == 1 else 7
        self.desired_joint_ang[start_idx:start_idx+7] = joint_angles

    def open_gripper(self, arm_number):
        gripper_idx = 14 if arm_number == 1 else 15
        self.desired_joint_ang[gripper_idx] = self.open_gripper_angle

    def close_gripper(self, arm_number):
        gripper_idx = 14 if arm_number == 1 else 15
        self.desired_joint_ang[gripper_idx] = self.closed_gripper_angle

    def go_to_desired(self, model, data):
        mj.set_mjcb_control(self.PID_control)