import mujoco as mj
from mujoco import viewer
import numpy as np
import math

# Set the XML filepath
xml_filepath = "/home/megan/magic/magic_manip/kinova_gen3_mujoco/two_arm_table.xml"

def PID_control(model, data):

 # Basic PD control gains
    Kp = 100.0  # Position gain - start with a low value
    Kd = 10.0   # Damping gain
    
    # Compute control torques (gravity compensation + position control)
    gravity = data.qfrc_bias[:14]  # Gravity compensation terms
    position_error = data.qpos[:14] - current_pos  # Try to maintain current position
    velocity_damping = Kd * data.qvel[:14]  # Damping term to reduce oscillations
    
    # Combine terms
    data.ctrl[:14] = gravity - Kp * position_error - velocity_damping

if __name__ == "__main__":
    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)

    # Set the simulation scene to the home configuration
    mj.mj_resetDataKeyframe(model, data, 0)

    mj.set_mjcb_control(PID_control)

    # Launch the simulate viewer
    viewer.launch(model, data)