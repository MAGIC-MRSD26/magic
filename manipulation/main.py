import mujoco as mj
from mujoco import viewer
import numpy as np
from manipulation.one_arm_pid_control import SingleArmPIDControl

if __name__ == "__main__":

    # Set the XML filepath
    xml_filepath = "/home/megan/magic/manipulation/kinova_gen3_mujoco/two_arm_table.xml"

    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)
    model.opt.timestep = 0.001

    controller = SingleArmPIDControl(model, data)

    controller.go_to_home(model, data)

    # Set specific arm positions
    arm1_angles = [0, np.pi/4, 0, np.pi/2, 0, 0, 0]  # 7 joint angles
    controller.set_arm_target(1, arm1_angles)

    # Control grippers
    controller.close_gripper(1)  # close first gripper
    controller.close_gripper(2)
    controller.go_to_desired(model, data)
 

    # object_pos = np.array([
    #         [0.3, 0.0, 0.5, 0.0, 0.0, 0.0],
    #         [0.3, 0.0, 0.5, 0.0, 0.0, 0.0]
    #     ])
    # controller.go_to_desired(model, data, object_pos)

    '''
    Workflow:

    Take in desired goal point in world frame for robot 1 and robot 2
    Do IK on each robot arm depending on goal point
    use SingleArmControl class for each arm to reach grasp point and close gripper

    ***Now the two robots can be treated as one system with 14 DoF
    take in desired height
    IK
    lift object using DualArmControl class
    Take in desired orientation of object
    do some massaging to get it in cartesian 
    IK
    use DualArmControl class for mid air manpulation

    '''

    # Launch the simulate viewer
    viewer.launch(model, data)