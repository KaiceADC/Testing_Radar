import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kaice/Desktop/radar_tracking_pkg/install/radar_tracking_pkg'
