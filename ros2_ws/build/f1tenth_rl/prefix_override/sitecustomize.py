import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/toyonishiorin/f1tenth-project/ros2_ws/install/f1tenth_rl'
