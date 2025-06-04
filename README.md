# RoboEntrega - Equipo 1

## Instrucciones
### Conectarse al robot real
Para conectarnos al robot real debemos primero conectarnos a su misma red wifi, en nuestro caso como estamos usando el robot 11 la red es **TP-LINK_6CAE**.\
Comprobamos que desde la máquina virtual tenemos conexion y que nuestra ip comienza por 192.168.0., se puede comprobar con el comando **ifconfig**.\
Ahora nos conectamos a nuestro robot poniendo en el cmd:
```
[Terminal de conexión]
ssh ubuntu@192.168.0.136
```
Si se ha hecho bien pedirá la contraseña, esta es **turtlebot**.\
Para inicializar el robot se debe usar
```
ros2 launch turtlebot3_bringup robot.launch.py
```
> [!IMPORTANT]
> Aunque varias perosnas se conecten a la vez, solo UNA debera hacer esta última linea, de hacerla varios el robot crasheará, se reiniciará y habrá que iniciarlo todo de nuevo
> 
Tambien hay que asegurarse de que la id del robot sea el 1, ya que somos el equipo 1
```
export ROS_DOMAIN_ID=1 # ID del equipo
```
### Comandos para iniciar
#### ROS2
Como tenemos la norma de no subir al git los archivos de compilado, primero compilaremos con:
```
[Terminal1]
[~/robot]
colcon build --symlink-install
source install/setup.bash
```
Se recomienda, en vez de copiar y pegar los comandos, usar la linea **source install/setup.bash** al iniciar cada nuevo terminal y escribir los comandos usando el tabulador para asegurar que esta bien escrito el comando, siempre puede haber algun error ortográfico
Abrimos gazebo para la simulación en caso de que se vaya a trabajar en la simulación y no con el robot real
```
[Terminal1]
ros2 launch robo_entrega_office_world office_world.launch.py
```
Despues abrimos al rviz y la detección de objetos donde ya se inicia el mapa, el modelo del robot y permite que comience la navegación y la deteccion de cajas y personas
Puede tardar varios segundos en abrirlo todo y poder empezar a usar la navegación.
```
[Terminal2]
ros2 launch robo_entrega_nodes_launcher project_launcher.launch.py
```
> [!TIP]
> Si en vez de ejecutar la lina anterior solo se quiere ejecutar una parte, podeis ejecutar los nodos por separado usando estos comandos en terminales diferentes:
> ```
> ros2 launch robo_entrega_nav2_system tb3_sim_nav2.launch.py
> ros2 run robo_entrega_capture_image detectar_caja
> ```
Y por último en este apartado haremos que el robot comience la ruta por la oficina
```
[Terminal3]
ros2 run robo_entrega_nav2_system my_waypoint_follower
```
#### WEB
> [!IMPORTANT]
> Es necesario ejecutar estos comandos al menos una vez en un terminal para asegurar que puedas ejecutar el servidor
> ```
> [Terminal nuevo]
> [~/web/backend]
> pip install -r requirements.txt
> ```
\
Primero lanzamos el puente entre web y ros
```
[Terminal4]
[~/web]
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Despues lanzaremos el servidor de video para la funcionalidad de la cámara
```
[Terminal5]
[~/robot]
ros2 run robo_entrega_web_video_server robo_entrega_video_server
```
Ahora abrimos el servidor en el puerto 8000 para poder ver la web
```
[Terminal6]
[~/web]
python3 -m http.server 8000
```
Despues ejecutamos el backend para poder usar sus funcionalidades con la base de datos
```
[Terminal7]
[~/web/backend]
python3 app.py
```
Por último ejecutamos un último nodo de ros2 para que el robot pueda hacer el recorrido de las entregas al conectarse a la base de datos
```
[Terminal8]
[~/robot]
ros2 launch robo_entrega_web_requester web_requester.launch.py
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

> [!IMPORTANT]
> Hay un usuario admin ya creado; \
> correo: admin@gmail.com \
> Contraseña: admin

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
