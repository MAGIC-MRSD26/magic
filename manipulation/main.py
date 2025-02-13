import mujoco as mj
from mujoco import viewer
import numpy as np
from one_arm_control import SingleArmControl

if __name__ == "__main__":

    # Set the XML filepath
    xml_filepath = "/home/megan/magic/manipulation/kinova_gen3_mujoco/two_arm_table.xml"

    # Load the xml file here
    model = mj.MjModel.from_xml_path(xml_filepath)
    data = mj.MjData(model)

    controller = SingleArmControl(model, data)

    controller.goToHome()

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