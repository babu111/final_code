import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zeyi19@netid.washington.edu/ningbo_ws/src/final_code/install/final_code'
