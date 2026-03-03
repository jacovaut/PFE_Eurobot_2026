import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/com2001/PFE_Eurobot_2026/ws/src/install/pfe'
