import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zguo27/auvc_ws/src/AUVC_stuff/install/bluerov2_testers'
