import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alaeddine/CS4048_SheepHerding/install/sheep_simulation'
