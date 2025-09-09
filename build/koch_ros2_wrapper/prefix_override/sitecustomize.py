import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chengwsam/ros2_ws/src/install/koch_ros2_wrapper'
