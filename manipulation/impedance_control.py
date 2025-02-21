import mujoco as mj
from mujoco import viewer
import numpy as np
import math
import utils

class ImpedanceControl():

    def __init__(self, model, data, desired_point):

        self.closed_gripper_angle = 0.725 #rad
        self.open_gripper_angle = 0.0 #rad

        # Cartesian space position and velocity
        self.pos_cartesian_curr = np.zeros(6)
        self.vel_cartesian_curr = np.zeros(6)

        self.hand_id = {
            1: model.body('hand').id,
            2: model.body('hand_2').id
        }

        self.desired_point = desired_point


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
            position_error = self.desired_point[0 if arm == 1 else 1] - self.pos_cartesian_curr

            # Get velocity error
            desired_velocity = np.zeros(6)
            velocity_error = desired_velocity - self.vel_cartesian_curr 

            # Calculate Jacobian
            jacp = np.zeros((3, model.nv)) # Translational Jacobian
            jacr = np.zeros((3, model.nv)) # Rotational Jacobian
            mj.mj_jac(model, data, jacp, jacr, data.xpos[hand_id], hand_id)
            J = np.vstack([jacp[:3, :7], jacr[:3, :7]])

            # Set default gains
            D_d = np.diag([5, 5, 5, 5, 5, 5])  # Damping 
            K_d = np.diag([100, 100, 100, 100, 100, 100])  # Stiffness

            # Compute impedance forces and torques
            impedance_force = D_d @ velocity_error + K_d @ position_error

            # Compute joint torques
            torque = J.T @ impedance_force

            # Before applying torques, saturate them
            max_torque = 5.0  # Start very conservative
            torque = np.clip(torque, -max_torque, max_torque)

            # Set joint angles for the specified arm
            start_idx = 0 if arm == 1 else 7
            data.ctrl[start_idx:start_idx+7] = data.qfrc_bias[start_idx:start_idx+7] + torque


    def get_ee_cartesian(self, data, arm):

        # Get position and orientation
        pos = data.xpos[self.hand_id[arm]]
        rot = data.xquat[self.hand_id[arm]]  # Quaternion [w, x, y, z]
        
        # Convert quaternion to euler angles (xyz convention)
        euler = utils.quat2euler(rot)
        
        # Combine position and orientation into pose
        self.pos_cartesian_curr  = np.concatenate([pos, euler])
        
        # Get linear and angular velocity
        lin_vel = data.cvel[self.hand_id[arm]][:3]  # Linear velocity in local frame
        ang_vel = data.cvel[self.hand_id[arm]][3:]  # Angular velocity in local frame
        
        # Transform velocities to world frame
        rot_mat = utils.quat2mat(rot)
        lin_vel_world = rot_mat @ lin_vel
        ang_vel_world = rot_mat @ ang_vel
        
        # Combine into spatial velocity
        self.vel_cartesian_curr  = np.concatenate([lin_vel_world, ang_vel_world])

    def go_to_desired(self, model, data):
        
        # Set the callback function
        mj.set_mjcb_control(self.impedance_control)