import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alaeddine/sheep_herding_ws/install/sheep_simulation'
