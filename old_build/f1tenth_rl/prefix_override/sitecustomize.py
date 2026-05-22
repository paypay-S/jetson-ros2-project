import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/toyonishiorin/projects/f1tenth-project/install/f1tenth_rl'
