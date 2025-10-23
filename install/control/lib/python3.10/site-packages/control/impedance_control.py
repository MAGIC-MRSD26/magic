import mujoco as mj
from mujoco import viewer
import numpy as np
import math
import utils
import quaternion

class ImpedanceControl():

    def __init__(self, model, data, desired_point):

        # Set desired point in world frame as an array or 2 arrays, one for each arm
        self.desired_point = desired_point

        self.closed_gripper_angle = 0.725 #rad
        self.open_gripper_angle = 0.0 #rad

        # Cartesian space position and velocity
        self.pos_cartesian_curr = np.zeros(6)
        self.vel_cartesian_curr = np.zeros(6)

        self.hand_id = {
            1: model.body('hand').id,
            2: model.body('hand_2').id
        }

    def impedance_control(self, model, data):

        '''
        inputs:
        desired_pos = an array of 2 arrays containing the desired end effector position for arm1 and arm2 
        '''

        # open gripper
        data.ctrl[14] = self.open_gripper_angle
        data.ctrl[15] = self.open_gripper_angle

        for arm, hand_id in self.hand_id.items():

            # Find current end effector position and velocity in Cartesian space (aka task space, world frame)
            self.get_ee_cartesian(data, arm)

            # Get position error
            position_error = self.desired_point[arm - 1] - self.pos_cartesian_curr

            # Calculate Jacobian
            start_jac_idx = 0 if arm == 1 else 13
            jacp = np.zeros((3, model.nv)) # Translational Jacobian
            jacr = np.zeros((3, model.nv)) # Rotational Jacobian
            mj.mj_jac(model, data, jacp, jacr, data.xpos[hand_id], hand_id)
            J = np.vstack([jacp[:, start_jac_idx:start_jac_idx+7], jacr[:, start_jac_idx:start_jac_idx+7]])

            # Get velocity error
            desired_velocity = np.zeros(6)
            start_vel_idx = 0 if arm == 1 else 13
            self.vel_cartesian_curr = J @ data.qvel[start_vel_idx:start_vel_idx+7]
            velocity_error = desired_velocity - self.vel_cartesian_curr 

            # Set default gains
            K_d = np.diag([60, 60, 60, 13, 13, 13])
            D_d = np.diag([4, 4, 4, 2, 2, 2])

            # Compute impedance forces and torques
            impedance_force = D_d @ velocity_error + K_d @ position_error

            # Compute joint torques
            torque = J.T @ impedance_force

            # constrain torque
            max_torque = 10
            torque = np.clip(torque, -max_torque, max_torque)

            # Set joint angle control for the specified arm with gravity
            start_idx = 0 if arm == 1 else 7

            data.ctrl[start_idx:start_idx+7] = data.qfrc_bias[start_vel_idx:start_vel_idx+7] + torque
        
            print(f"Arm {arm} - Current: {self.pos_cartesian_curr[:3]}, Desired: {self.desired_point[arm-1][:3]}, Error: {position_error[:3]}")

    def get_ee_cartesian(self, data, arm):

        # Get position and orientation
        pos = data.xpos[self.hand_id[arm]]
        rot = data.xquat[self.hand_id[arm]]  # Quaternion [w, x, y, z]

        # Convert quaternion to euler angles (xyz convention)
        axis, angle = utils.quat2axisangle(rot)
        axisangle = axis * angle
        
        # Combine position and orientation into pose
        self.pos_cartesian_curr  = np.concatenate([pos, axisangle])

    def go_to_desired(self, model, data):
        
        # Set the callback function
        mj.set_mjcb_control(self.impedance_control)