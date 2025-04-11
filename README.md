# RoboEntrega - Equipo 1

## Instrucciones
### Comandos para iniciar

> [!WARNING]
> El nombre de paquete de `my_world` será cambiado en el proximo sprint a un nombre más apropiado

Como tenemos la norma de no subir al git los archivos de compilado, primero compilaremos con:
```
colcon build --symlink-install
source install/setup.bash
```
Luego abriremos gazebo
```
ros2 launch my_world turtlebot3_my_world.launch.py
```
Despues abrimos al rviz sonde ya se inicia el mapa, el modelo del robot y permite que comience la navegación
```
ros2 launch nav2_system tb_sim_nav2.launch.py
```
Puede tardar varios segundos en abrirlo todo y poder empezar a usar la navegación.

## Nuestro proyecto
### Quienes somos?
Somos un grupo de alumnos de 3º del [Grado en tecnologias interactivas (GTI)](https://www.upv.es/titulaciones/GTI/) en la escuela politécnica superior de Gandia de la **Universidad Politécnica de Valencia**.\
Somos: 
- [Diego Baldoví](https://github.com/Foxpeet)
- [Hao Xu](https://github.com/Hao12341)
- [Endika Matute](https://github.com/EndikaMB1)
- [Alejandro Roca](https://github.com/Roca057)
- [Jaime Ferrer](https://github.com/JaumeFerrer)
- [Jordan Phillips](https://github.com/Masterboy272)
### De que va nuestro proyecto?
Durante este segundo cuatrimestre del 3º curso realizaremos como proyecto de la asignatura de Robotica, un robot repartidor que sea capaz de reconocer el destinatario de una carta, paquete, documento, etc. Y sea capaz de llevar ese bulto al puesto de trabajo del destinatario o incluso buscarla por la oficina, con el objetivo de hacer más eficiente y rápido el trabajo de los empleados de la oficina mediante el control del robot mediante una página web
### Como vamos a hacerlo?
Programaremos el robot que se nos ha proporcionado en clase, un **turtlebot3 modelo burger**, en ROS2 usando una máquina virtual donde tenemos instalado un **Linux Ubuntu**.
### Nuestro robot
Nuestro robot es un [turtlebot3 modelo burger](https://www.turtlebot.com/turtlebot3/).\
En proximos sprints diseñaremos e imprimiremos piezas para el turtlebot con la intencion de hacerlo mas eficiente en el reparto de paquetes y cartas
### Nuestro mapa
Nuestro mapa/espacio de simulacion donde el robot realizará las pruebas y testeos es una oficina creada por [Melih Erdogan](https://github.com/mlherd) en su repositorio [Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps).
Modificamos su mapa para no consumir tantos recursos en nuestra **máquina virtual Linux Ubuntu**.\
Así resultó:
![imagen del mapa de la oficina de gazebo](https://i.imgur.com/s6jIldi.png)
