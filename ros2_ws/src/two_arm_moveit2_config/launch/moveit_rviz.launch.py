from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gen3_robotiq_2f_85", package_name="two_arm_moveit2_config").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
