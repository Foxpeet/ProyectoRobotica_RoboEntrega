import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotica/proyectoRoboentrega/ProyectoRobotica_RoboEntrega/robot/install/robo_entrega_slam'
