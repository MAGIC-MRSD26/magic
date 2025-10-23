import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emma/Documents/GitHub/magic/install/rqt_controller_manager'
