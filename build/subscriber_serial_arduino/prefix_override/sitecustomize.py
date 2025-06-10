import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/manika/Desktop/LTU/Filip/drone_ws/install/subscriber_serial_arduino'
