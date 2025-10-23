import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emma/Documents/GitHub/magic/install/ros2_controllers_test_nodes'
