// función de la conexión con ROS2 -----------------------------------------
function connect(){
    console.log("Clic en connect")

    data.ros = new ROSLIB.Ros({
          url: data.rosbridge_address
  })

  // Define callbacks
  data.ros.on("connection", () => {
      data.connected = true
      console.log("Conexion con ROSBridge correcta")
      //Activar el boton de enviar coordenadas una vez se ha conectado ros2
      document.getElementById('sendCoordinatesButton').disabled = false;
      //actualizamos una sola vez la cámara
      updateCameraFeed()
      iniciarSubscripcionMapa()
  })
  data.ros.on("error", (error) => {
      console.log("Se ha producido algun error mientras se intentaba realizar la conexion")
      console.log(error)
  })
  data.ros.on("close", () => {
      data.connected = false
      console.log("Conexion con ROSBridge cerrada")
      //Deshabilitar el boton de enviar una vez se desconecta de ros2
      document.getElementById('sendCoordinatesButton').disabled = true;
  })
}

function disconnect(){
    data.ros.close()
    data.connected = false
    console.log('Clic en botón de desconexión')
}

// función de la cámara -----------------------------------------
function updateCameraFeed() {
    const img = document.getElementById("cameraFeed");
    const timestamp = new Date().getTime(); // Evita caché agregando un timestamp
    img.src = `http://0.0.0.0:8080/stream?topic=/camera/image_raw`;
    //img.src = `http://0.0.0.0:8080/stream?topic=/deteccion/detecta_caja`;
    //img.src = `http://localhost:8080/stream?topic=/turtlebot3/camera/image_raw&console.log("Cactualizando: http://0.0.0.0:8080/stream?topic=/camera/image_raw)"`
}

// función del mapa -----------------------------------------
const mapaImagen = new Image();
mapaImagen.src = "../img/mapa.png"; // Asegúrate de que este archivo exista en tu carpeta /img/

const MAP_RESOLUTION = 0.05; // metros por pixel
const MAP_ORIGIN = { x: -3.64, y: -28.5 }; // origen del mundo real

mapaImagen.onload = () => {
    const canvas = document.getElementById("mapCanvas");
    const ctx = canvas.getContext("2d");
    ctx.drawImage(mapaImagen, 0, 0, canvas.width, canvas.height);
};

function iniciarSubscripcionMapa() {
    const listener = new ROSLIB.Topic({
        ros: data.ros,
        name: '/amcl_pose',
        messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });

    listener.subscribe((message) => {
        const x = message.pose.pose.position.x;
        const y = message.pose.pose.position.y;
        const theta = getYawFromQuaternion(message.pose.pose.orientation);
        dibujarRobotEnMapa(x, y, theta);
    });
}

function getYawFromQuaternion(orientation) {
    const { x, y, z, w } = orientation;
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    return Math.atan2(siny_cosp, cosy_cosp); // radianes
}

function worldToCanvasCoords(x, y) {
    const canvas = document.getElementById("mapCanvas");
    const mapWidth = mapaImagen.width;
    const mapHeight = mapaImagen.height;

    const px = (x - MAP_ORIGIN.x) / MAP_RESOLUTION;
    const py = (y - MAP_ORIGIN.y) / MAP_RESOLUTION;

    const scaleX = canvas.width / mapWidth;
    const scaleY = canvas.height / mapHeight;

    return {
        x: px * scaleX,
        y: canvas.height - (py * scaleY)
    };
}

function dibujarRobotEnMapa(x, y, theta) {
    const canvas = document.getElementById("mapCanvas");
    if (!canvas || !mapaImagen.complete) return;

    const ctx = canvas.getContext("2d");
    ctx.drawImage(mapaImagen, 0, 0, canvas.width, canvas.height); // Redibuja fondo

    const { x: cx, y: cy } = worldToCanvasCoords(x, y);

    ctx.beginPath();
    ctx.arc(cx, cy, 8, 0, 2 * Math.PI);
    ctx.fillStyle = "red";
    ctx.fill();

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + 15 * Math.cos(theta), cy - 15 * Math.sin(theta));
    ctx.strokeStyle = "black";
    ctx.stroke();
}


// funciones de las flechas y el controlador -----------------------------------------

function enviarCoordenadas(x, y, w) {
    if (!data.connected) {
        console.log("No hay conexión con ROSBridge");
        return;
    }

    const publisher = new ROSLIB.Topic({
        ros: data.ros,
        name: '/navigate_goal',
        messageType: 'std_msgs/Float32MultiArray'
    });

    const message = new ROSLIB.Message({
        data: [x, y, w]
    });

    publisher.publish(message);
    console.log(`Coordenadas enviadas: x=${x}, y=${y}, w=${w}`);
}


// asignación de escuchadores -----------------------------------------
document.addEventListener('DOMContentLoaded', event => {
    data = {
        ros: null,
        rosbridge_address: 'ws://127.0.0.1:9090/',
        connected: false,
    };

    connect();

    // Escuchador para el formulario de coordenadas
    document.getElementById('coordinateForm').addEventListener('submit', (event) => {
        event.preventDefault();
    
        const x = parseFloat(document.getElementById('x').value);
        const y = parseFloat(document.getElementById('y').value);
        const w = parseFloat(document.getElementById('w').value);
    
        if (isNaN(x) || isNaN(y) || isNaN(w)) {
            console.error("Valores inválidos ingresados.");
            alert("Por favor, ingresa valores numéricos válidos para X, Y y W.");
            return;
        }
    
        enviarCoordenadas(x, y, w);
    });

    data.ros.on("close", () => {
        data.connected = false;
        console.log("Conexion con ROSBridge cerrada. Intentando reconectar en 5 segundos...");
        document.getElementById('sendCoordinatesButton').disabled = true;
    
        setTimeout(() => {
            connect();
        }, 5000);
    });

});


