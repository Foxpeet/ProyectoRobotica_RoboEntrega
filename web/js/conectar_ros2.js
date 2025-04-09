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
      //actualizamos una sola vez la cámara
      updateCameraFeed()
  })
  data.ros.on("error", (error) => {
      console.log("Se ha producido algun error mientras se intentaba realizar la conexion")
      console.log(error)
  })
  data.ros.on("close", () => {
      data.connected = false
      console.log("Conexion con ROSBridge cerrada")
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
    //img.src = `http://localhost:8080/stream?topic=/turtlebot3/camera/image_raw&console.log("Cactualizando: http://0.0.0.0:8080/stream?topic=/camera/image_raw)"`
}

// función del mapa -----------------------------------------


// funciones de las flechas y el controlador -----------------------------------------



// asignación de escuchadores -----------------------------------------
document.addEventListener('DOMContentLoaded', event => {
    data = {
        ros: null,
        rosbridge_address: 'ws://127.0.0.1:9090/',
        connected: false,
    }
    connect()
});

window.addEventListener('beforeunload', event => {
    disconnect()
})
