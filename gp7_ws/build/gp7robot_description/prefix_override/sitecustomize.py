import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lakindugunasekara/gp7_ws/install/gp7robot_description'
