import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ake/nav/nav_neupan/src/bot_nav/install/robot_patrol'
