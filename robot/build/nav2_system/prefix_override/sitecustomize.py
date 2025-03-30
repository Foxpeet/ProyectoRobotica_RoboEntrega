import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/haoxu/Git/ProyectoRobotica_RoboEntrega/robot/install/nav2_system'
