# #!/usr/bin/env python3

# import os
# import sys

# # First, ensure mujoco is imported properly
# try:
#     import mujoco
#     print(f"MuJoCo imported successfully! Version: {mujoco.__version__}")
# except ImportError as e:
#     print(f"ERROR: Failed to import mujoco: {e}")
#     print("Please install MuJoCo: pip install mujoco")
#     sys.exit(1)

# # Now check the ROS2 environment and packages
# try:
#     from ament_index_python.packages import get_package_share_directory
#     print("ament_index_python imported successfully!")
# except ImportError as e:
#     print(f"ERROR: Failed to import ament_index_python: {e}")
#     print("Make sure your ROS2 environment is properly sourced.")
#     print("source /opt/ros/<your-ros-distro>/setup.bash")
#     sys.exit(1)

# # Check if the control package exists
# try:
#     control_package_path = get_package_share_directory("two_arm_moveit2_config")
#     print(f"Found control package at: {control_package_path}")
# except Exception as e:
#     print(f"ERROR: Could not find 'control' package: {e}")
#     print("Make sure the 'control' package is installed and built.")
#     sys.exit(1)

# # Check the model path
# model_path = os.path.join(
#     control_package_path,
#     "mujoco_model",
#     "dual_arm_setup.xml"
# )

# print(f"Looking for model at: {model_path}")

# if not os.path.exists(model_path):
#     print(f"ERROR: Model file not found at {model_path}")
    
#     # Check if the directory exists
#     dir_path = os.path.dirname(model_path)
#     if not os.path.exists(dir_path):
#         print(f"Directory not found: {dir_path}")
#         parent_dir = os.path.dirname(dir_path)
#         if os.path.exists(parent_dir):
#             print(f"Available files in parent directory {parent_dir}:")
#             for item in os.listdir(parent_dir):
#                 print(f"  - {item}")
#     else:
#         print(f"Available files in directory {dir_path}:")
#         for item in os.listdir(dir_path):
#             print(f"  - {item}")
    
#     sys.exit(1)

# print(f"Model file found! Size: {os.path.getsize(model_path)} bytes")

# # Try to load the model
# try:
#     print("Attempting to load the model...")
#     model = mujoco.MjModel.from_xml_path(model_path)
#     data = mujoco.MjData(model)
#     print("Model loaded successfully!")
    
#     # Print model stats
#     print(f"Model stats:")
#     print(f"  - Position degrees of freedom (nq): {model.nq}")
#     print(f"  - Velocity degrees of freedom (nv): {model.nv}")
#     print(f"  - Actuators/controls (nu): {model.nu}")
#     print(f"  - Bodies (nbody): {model.nbody}")
#     print(f"  - Joints (njnt): {model.njnt}")
#     print(f"  - Geometries (ngeom): {model.ngeom}")
    
#     # Try stepping the simulation
#     print("Attempting to step the simulation...")
#     mujoco.mj_step(model, data)
#     print("Simulation stepped successfully!")
    
# except Exception as e:
#     print(f"ERROR loading model: {e}")
    
#     # Try to inspect the file content
#     try:
#         with open(model_path, 'r') as f:
#             content = f.read(1000)  # Read first 1000 characters
#             print("\nFirst 1000 characters of the model file:")
#             print(content)
#             print("...")
#     except Exception as file_e:
#         print(f"Could not read file content: {file_e}")
    
#     sys.exit(1)

# print("\nAll checks passed successfully! Your MuJoCo model is valid.")

import mujoco
import mujoco.viewer
import os
from ament_index_python.packages import get_package_share_directory

model_path = os.path.join (
    get_package_share_directory("two_arm_moveit2_config"),
    "mujoco_model",
    "gen3_dual_arm.xml",)

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()