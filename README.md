# RoboEntrega - Equipo 1

## Instrucciones
### Comandos para iniciar
#### ROS2
Como tenemos la norma de no subir al git los archivos de compilado, primero compilaremos con:
```
[Terminal1]
[~/robot]
colcon build --symlink-install
source install/setup.bash
```
Abrimos gazebo para la simulación
```
[Terminal1]
ros2 launch robo_entrega_office_world office_world.launch.py
```
Luego abrimos el nodo de la acción de navegación
```
[Terminal2]
ros2 launch robo_entrega_nav_to_pose nav_to_pose_subscriber.launch.py
```
Despues abrimos al rviz sonde ya se inicia el mapa, el modelo del robot y permite que comience la navegación
```
[Terminal3]
ros2 launch robo_entrega_nav2_system tb3_sim_nav2.launch.py
```
Puede tardar varios segundos en abrirlo todo y poder empezar a usar la navegación.
\
#### WEB
Primero lanzamos el puente entre web y ros
```
[Terminal4]
[~/web]
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Despues lanzaremos el servidor de video para la funcionalidad de la cámara
```
[Terminal5]
ros2 run robo_entrega_web_video_server robo_entrega_video_server
```
Por último creamos el servidor en el puerto 8000 para poder ver la web
```
[Terminal6]
python3 -m http.server 8000
```

### probar manualmente el nav_to_pose
```
[Terminal nuevo]
ros2 topic pub --once /navigate_goal std_msgs/msg/Float32MultiArray "{data: [2.0, 3.0, 90.0]}"
```
> [!TIP]
> Asegurate de que tienes estos paquetes actualizados para poder ejecutar correctamente el apartado de WEB.
> ```
> sudo apt update
> sudo apt install ros-galactic-rosbridge-suite
> pip install numpy
> pip install pyaml
> pip install netifaces
> pip install pymongo
> pip install pillow
> pip install tornado
> ```

## Nuestro proyecto
### Como vamos a organizar el git?
Al crear el repositorio tenemos una rama `main` que contendrá el codigo totalmente funcional que presentaremos en cada *Sprint review*, de esta rama `main` se crean ramas `release0X` al principio de sprint donde *X* es el número del sprint que trabajaremos y que se cerrará al finalizar un sprint y por último de ésta rama `release0X` se crearán ramas `QT-HX-YY` donde *YY* es el número de tarea y *X* el número de la historia de usuario a la que corresponde esa tarea, y que representará el trabajo de una tarea, al terminar la tarea la rama se cerrara en `release0X`. \
Nuestros commits solo se harán en las ramas tarea (`QT-HX-YY`) y en casos estrictamente necesarios e irremediables en la rama sprint (`release0X`). \
![imagen de la distribución de ramas de git](https://i.imgur.com/1QnHS7a.png)

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
