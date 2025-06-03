
let robots = [];
let workers = [];
let desks = [];

let nextRobotId = 1;
let nextWorkerId = 1;
let nextDeskId = 1;

const robotForm = document.getElementById("robotForm");
const robotTable = document.getElementById("robotTable");

const workerForm = document.getElementById("workerForm");
const workerTable = document.getElementById("workerTable");

const deskForm = document.getElementById("deskForm");
const deskTable = document.getElementById("deskTable");

// MODALES
const editWorkerModal = document.getElementById("editWorkerModal");
const closeEditWorkerModal = document.getElementById("closeEditWorkerModal");
const editWorkerForm = document.getElementById("editWorkerForm");

const changeDeskModal = document.getElementById("changeDeskModal");
const closeChangeDeskModal = document.getElementById("closeChangeDeskModal");
const deskSelect = document.getElementById("deskSelect");
const deskChange = document.getElementById("deskChange");
const confirmChangeDeskBtn = document.getElementById("confirmChangeDeskBtn");

const API_URL = 'http://127.0.0.1:5000/api';

document.addEventListener('DOMContentLoaded', () => {
   usuario = sessionStorage.getItem('usuario');

  if (!usuario) {
      // Si no hay sesión activa, redirigir al inicio de sesión
      window.location.href = 'iniciarSesion.html';
      return;
  }

  const datosUsuario = JSON.parse(usuario);

  const nombre = datosUsuario.nombre_trabajador;
  const apellido = datosUsuario.apellido_trabajador;

  const bienvenida = document.getElementById('bievenida_admin');
  bienvenida.textContent = `¡Bienvenido, ${nombre}!`;


// Gestionar el cierre de sesión
const logoutLink = document.getElementById('header-logout-link');
logoutLink.addEventListener('click', async (e) => {
    e.preventDefault(); // Prevenir el comportamiento por defecto del enlace
    
    try {
        const response = await fetch('http://127.0.0.1:5000/api/logout', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                correo: datosUsuario.correo
            })
        });

        if (response.ok) {
            sessionStorage.removeItem('usuario');
            
            window.location.href = 'iniciarSesion.html';
        } else {
            const errorData = await response.json();
            console.error('Error al cerrar sesión:', errorData.error);
            alert('Ocurrió un error al cerrar sesión. Por favor, inténtalo de nuevo.');
        }
    } catch (error) {
        console.error('Error al conectar con el servidor:', error);
        alert('No se pudo conectar con el servidor. Verifica tu conexión a internet.');
    }
});
});


// --- FUNCIONES ROBOTS ---
async function renderRobots() {
  robotTable.innerHTML = "";
  const response = await fetch(`${API_URL}/robots`);
  const robots = await response.json();

  robots.forEach((r) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${r.id_robot}</td>
      <td>${r.modelo_robot}</td>
      <td>${r.numero_serie_robot}</td>
      <td>
        <button class="delete-button" title="Eliminar robot">
          <img src="img/basura_eliminar_boton.png" alt="Eliminar" />
        </button>
      </td>`;

    tr.querySelector(".delete-button").addEventListener("click", () => {
      openDeleteRobotModal(r); 
    });

    robotTable.appendChild(tr);
  });
}

function openDeleteRobotModal(robot) {
  const modal = document.getElementById("deleteRobotModal");
  const message = document.getElementById("deleteRobotMessage");
  const confirmBtn = document.getElementById("confirmDeleteRobotBtn");
  const cancelBtn = document.getElementById("cancelDeleteRobotBtn");

  message.textContent = `¿Eliminar robot ${robot.modelo_robot} ${robot.numero_serie_robot}?`;

  modal.style.display = "flex";

  // Confirmar eliminación
  confirmBtn.onclick = async () => {
    try {
      const deleteResponse = await fetch(`${API_URL}/robots/${robot.id_robot}`, {
        method: 'DELETE',
      });

      if (deleteResponse.ok) {
        mostrarExito(`Robot ID ${robot.id_robot} eliminado.`);
        renderRobots();
      } else {
        mostrarError("Error al eliminar el robot.");
      }

      closeDeleteRobotModal();
    } catch (error) {
      mostrarError("Hubo un error al eliminar el robot.");
    }
  };

  // Cancelar eliminación
  cancelBtn.onclick = () => {
    closeDeleteRobotModal();
  };

  document.getElementById("closeDeleteRobotModal").onclick = closeDeleteRobotModal;
}

function closeDeleteRobotModal() {
  const modal = document.getElementById("deleteRobotModal");
  modal.style.display = "none";
}

robotForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  const newRobot = {
    modelo_robot: document.getElementById('robotModel').value.trim(),
    numero_serie_robot: document.getElementById("robotSerial").value.trim(),
  };

  try {
    const response = await fetch(`${API_URL}/robots`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(newRobot),
    });

    if (response.ok) {
      renderRobots();
      mostrarExito('Robot añadido correctamente', 'robotForm');
    } else {
      mostrarError(`Error añadiendo robot: ${response.statusText}`);
    }
  } catch (error) {
    mostrarError(`Error añadiendo robot: ${error}`);
  }
});

// --- FUNCIONES TRABAJADORES ---
async function renderWorkers() {
  try {
    const response = await fetch(`${API_URL}/trabajadores`);
    const trabajadores = await response.json();

    const table = document.getElementById("workerTable");
    table.innerHTML = "";

    trabajadores.forEach((t) => {
      const tr = document.createElement("tr");

      tr.innerHTML = `
        <td>${t.dni_trabajador}</td>
        <td>${t.nombre_trabajador}</td>
        <td>${t.apellido_trabajador}</td>
        <td>${t.correo}</td>
        <td>${t.mesa_id_mesa}</td>
        <td class="worker-actions">
          <button class="edit-button" title="Editar trabajador">
            <img src="img/lapiz_edit_button.png" alt="Editar" />
          </button>
          <button class="delete-button" title="Eliminar trabajador">
            <img src="img/basura_eliminar_boton.png" alt="Eliminar" />
          </button>
          <button class="btn-azul change-button" title="Asignar cabina">Cabina</button>
        </td>
      `;

      // Botón editar
      tr.querySelector(".edit-button").addEventListener("click", () => {
        openEditWorkerModal(t);
      });

      // Botón eliminar con gestión de concurrencia
      tr.querySelector(".delete-button").addEventListener("click", async () => {
        const confirmado = confirm(`¿Eliminar trabajador ${t.nombre_trabajador} ${t.apellido_trabajador}?`);

        if (!confirmado) return;

        await deleteWorker(t.dni_trabajador);
      });

      // Botón asignar cabina
      tr.querySelector(".change-button").addEventListener("click", () => {
        openChangeDeskModal(t);
      });

      table.appendChild(tr);
    });
  } catch (err) {
    mostrarError("Error cargando trabajadores", "error");
  }
}



// --- FUNCIONES TRABAJADORES ---
async function renderWorkers() {
  try {
    const response = await fetch(`${API_URL}/trabajadores`);
    if (!response.ok) {
      throw new Error('Error al obtener los trabajadores');
    }
    const trabajadores = await response.json();

    const table = document.getElementById('workerTable');
    table.innerHTML = "";
    
    trabajadores.forEach((t) => {
      const tr = document.createElement("tr");

      tr.innerHTML = `
        <td>${t.dni_trabajador}</td>
        <td>${t.nombre_trabajador}</td>
        <td>${t.apellido_trabajador}</td>
        <td>${t.correo}</td>
        <td>${t.mesa_id_mesa}</td>
        <td>
          <button class="edit-button" title="Editar trabajador">
            <img src="img/lapiz_edit_button.png" alt="Editar" />
          </button>
          <button class="delete-button" title="Eliminar trabajador">
            <img src="img/basura_eliminar_boton.png" alt="Eliminar" />
          </button>
          <button class="btn-azul change-button" title="Asignar cabina">Cabina</button>
        </td>
      `;

      // Botón editar
      tr.querySelector(".edit-button").addEventListener("click", () => {
        openEditWorkerModal(t);
      });

      // Botón eliminar
      tr.querySelector(".delete-button").addEventListener("click", async () => {
        const confirmado = confirm(`¿Eliminar trabajador ${t.nombre_trabajador} ${t.apellido_trabajador}?`);
        if (!confirmado) return;
        await deleteWorker(t.dni_trabajador);
      });
      
      // Asignar cabina
      tr.querySelector(".change-button").addEventListener("click", () => {
        openChangeDeskModal(t);
      });

      table.appendChild(tr);
    });
  } catch (err) {
    mostrarError("Error cargando trabajadores", "error");
  }
}

async function deleteWorker(dni) {
  try {
    const response = await fetch(`${API_URL}/trabajadores/${dni}`, {
      method: 'DELETE'
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.message || 'No se pudo eliminar el trabajador.');
    }

    mostrarExito(`Trabajador eliminado correctamente`, 'success');
    renderWorkers();
  } catch (err) {
    mostrarError(`Error eliminando trabajador: ${err.message}`, 'error');
  }
}

async function loadDesks() {
  const deskSelect = document.getElementById("deskSelect");

  try {
    // Llamar a la API para obtener las mesas y su estado
    const response = await fetch(`${API_URL}/mesas_con_trabajadores`);
    
    if (!response.ok) {
      const errorDetails = await response.text();
      throw new Error(`Error al obtener las mesas: ${response.status} - ${errorDetails}`);
    }

    const desks = await response.json();

    // Limpiar las opciones del select antes de agregar las nuevas (excepto el placeholder)
    deskSelect.innerHTML = '';
    const placeholder = document.createElement('option');
    placeholder.value = '';
    placeholder.disabled = true;
    placeholder.selected = true;
    placeholder.textContent = "Elige una mesa";
    deskSelect.appendChild(placeholder);

    if (desks.length === 0) {
      const option = document.createElement("option");
      option.textContent = "No hay cabinas disponibles";
      option.disabled = true;
      deskSelect.appendChild(option);
    } else {

      // Recorrer todas las mesas y crear las opciones
      desks.forEach((desk) => {
        const option = document.createElement("option");
        option.value = desk.id_mesa;
        
        option.textContent = `Mesa - ${desk.id_mesa}`;
        
        if (desk.ocupada) {
          option.disabled = true;
          option.textContent += " (Ocupada)";
        }

        deskSelect.appendChild(option);
      });
    }
  } catch (error) {
    mostrarError("Hubo un problema al cargar las mesas. Ver consola para más detalles.");
  }
}

document.addEventListener('DOMContentLoaded', loadDesks);

workerForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  const nuevoTrabajador = {
    dni_trabajador: document.getElementById('workerDNI').value,
    nombre_trabajador: document.getElementById('workerName').value,
    apellido_trabajador: document.getElementById('workerLastName').value,
    correo: document.getElementById('workerEmail').value,
    contraseña: document.getElementById('workerPassword').value,
    id_mesa: document.getElementById('deskSelect').value 
  };

  if (!nuevoTrabajador.dni_trabajador || 
      !nuevoTrabajador.nombre_trabajador || 
      !nuevoTrabajador.apellido_trabajador || 
      !nuevoTrabajador.correo || 
      !nuevoTrabajador.contraseña) {
      mostrarError("Por favor, rellena todos los campos obligatorios");
      return;
  }

  if (!/^[0-9]{8}[A-Za-z]$/.test(nuevoTrabajador.dni_trabajador)) {
      mostrarError("El DNI debe tener 8 números y 1 letra");
      return;
  }

  if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(nuevoTrabajador.correo)) {
      mostrarError("Por favor, introduce un email válido");
      return;
  }

  if (!nuevoTrabajador.id_mesa) {
      mostrarError("Por favor, selecciona una mesa para el trabajador");
      return;
  }

  try {
      const response = await fetch(`${API_URL}/trabajadores`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(nuevoTrabajador)
      });

      if (response.ok) {
          renderWorkers();
          e.target.reset(); 
          mostrarExito('Trabajador añadido correctamente');
      } else {
          const responseData = await response.json();
          if (responseData.message.includes('DNI')) {
              mostrarError("El DNI ya está registrado.");
          } else if (responseData.message.includes('correo')) {
              mostrarError("El correo electrónico ya está registrado.");
          } else {
              mostrarError(`Error: ${responseData.message || 'Error desconocido'}`);
          }
      }
  } catch (error) {
      mostrarError(`Error añadiendo trabajador: ${error}`);
  }
});

// --- MODAL EDITAR TRABAJADOR ---
document.addEventListener('DOMContentLoaded', () => {
  const editWorkerModal = document.getElementById("editWorkerModal");
  const closeEditWorkerModal = document.getElementById("closeEditWorkerModal");
  const editWorkerForm = document.getElementById("editWorkerForm");

  window.openEditWorkerModal = function(t) {
    editWorkerModal.style.display = "flex";
    document.getElementById("displayWorkerDNI").textContent = t.dni_trabajador;
    document.getElementById("displayWorkerName").textContent = t.nombre_trabajador;
    document.getElementById("displayWorkerLastName").textContent = t.apellido_trabajador;
    document.getElementById("editWorkerEmail").value = t.correo;
    document.getElementById("editWorkerPassword").value = "";
  };

  closeEditWorkerModal.addEventListener("click", () => {
    editWorkerModal.style.display = "none";
  });
  editWorkerForm.addEventListener("submit", async (e) => {
    e.preventDefault();
    
    const formData = {
      dni_trabajador: document.getElementById("displayWorkerDNI").textContent,
      correo: document.getElementById("editWorkerEmail").value,
      contraseña: document.getElementById("editWorkerPassword").value
    };

    try {
      const response = await fetch(`${API_URL}/trabajadores/${formData.dni_trabajador}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData)
      });

      if (!response.ok) throw new Error('Error al actualizar');
      
      mostrarExito('Trabajador actualizado correctamente');
      editWorkerModal.style.display = "none";
      renderWorkers();
    } catch (error) {
      mostrarError(`Error: ${error.message}`, 'error');
    }
  });

  editWorkerModal.addEventListener("click", (e) => {
    if (e.target === editWorkerModal) {
      editWorkerModal.style.display = "none";
    }
  });
});


// --- MODAL CAMBIAR CABINA ---
let currentWorkerForDesk = null;

async function openChangeDeskModal(worker) {
  currentWorkerForDesk = worker;
  const changeDeskName = document.getElementById("changeDeskName");
  changeDeskName.textContent = `Cambiar Cabina para ${worker.nombre_trabajador} ${worker.apellido_trabajador}`;  changeDeskModal.style.display = "flex";  // Abrir el modal
  deskChange.innerHTML = "";
  const placeholder = document.createElement('option');
  placeholder.value = '';
  placeholder.disabled = true;
  placeholder.selected = true;
  placeholder.textContent = "Elige nueva mesa";
  deskChange.appendChild(placeholder);

  try {
    const response = await fetch(`${API_URL}/mesas_con_trabajadores`);
    if (!response.ok) {
      throw new Error("Error al obtener las mesas.");
    }

    const desks = await response.json();

    if (desks.length === 0) {
      const option = document.createElement("option");
      option.textContent = "No hay cabinas disponibles";
      option.disabled = true;
      deskChange.appendChild(option);
      confirmChangeDeskBtn.disabled = true;
    } else {
      let hasAvailableDesk = false;

      desks.forEach((desk) => {
        const option = document.createElement("option");
        option.value = desk.id_mesa;

        option.textContent = `Mesa - ${desk.id_mesa}`;

        if (desk.ocupada) {
          option.disabled = true;
          option.textContent += " (Ocupada)";
        } else {
          hasAvailableDesk = true;
        }
        deskChange.appendChild(option);
      });

      confirmChangeDeskBtn.disabled = !hasAvailableDesk;
    }
  } catch (error) {
    alert("Hubo un problema al cargar las mesas. Por favor, intenta nuevamente.");
  }
}


closeChangeDeskModal.addEventListener("click", () => {
  changeDeskModal.style.display = "none";
  currentWorkerForDesk = null;
});


confirmChangeDeskBtn.addEventListener("click", async () => {
  if (!currentWorkerForDesk) return;

  const selectedDeskId = parseInt(deskChange.value);

  if (!selectedDeskId) {
    alert("Por favor, selecciona una nueva mesa.");
    return;
  }

  try {
    const response = await fetch(`${API_URL}/trabajadores/${currentWorkerForDesk.dni_trabajador}/cambiar_mesa`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ id_mesa: selectedDeskId })
    });

    if (response.ok) {
      mostrarExito("La mesa ha sido cambiada con éxito.");
      changeDeskModal.style.display = "none";
      renderWorkers();  
    } else {
      const responseData = await response.json();
      mostrarError(`Error: ${responseData.message || 'No se pudo realizar el cambio de mesa.'}`);
    }
  } catch (error) {
    mostrarError("Hubo un error al intentar cambiar la mesa. Por favor, intenta nuevamente.");
  }
});



// --- FUNCIONES CABINAS CON API ---
async function renderDesks() {
  try {
    const response = await fetch(`${API_URL}/mesas`);
    
    if (!response.ok) {
      throw new Error(`Error HTTP: ${response.status}`);
    }
    
    const desks = await response.json();
        if (!Array.isArray(desks)) {
      throw new Error("Formato de respuesta inválido: se esperaba un array");
    }
    
    if (desks.length === 0) {
      deskTable.innerHTML = `
        <tr>
          <td colspan="3" class="no-data">
            No hay cabinas registradas
          </td>
        </tr>`;
      return;
    }
    
    deskTable.innerHTML = desks.map(desk => `
      <tr>
        <td>${desk.id_mesa}</td>
        <td>${desk.longitud_x}</td>
        <td>${desk.latitud_y}</td>
      </tr>
    `).join('');
        
  } catch (error) {
    mostrarError(` Error al cargar cabinas: ${error}`);
  }
}

document.addEventListener('DOMContentLoaded', () => {
  const form = document.getElementById('deskForm');

  form.addEventListener('submit', async (e) => {
    e.preventDefault();

    const nuevaMesa = {
      longitud_x: parseFloat(document.getElementById('deskLongitude').value),
      latitud_y: parseFloat(document.getElementById('deskLatitude').value)
    };

    if (isNaN(nuevaMesa.longitud_x) || isNaN(nuevaMesa.latitud_y)) {
      mostrarError("Por favor, introduce valores válidos para longitud y latitud.", "deskForm");
      return;
    }

    try {
      const response = await fetch(`${API_URL}/mesas`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(nuevaMesa)
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || "Error desconocido al añadir mesa");
      }

      mostrarExito("Mesa añadida correctamente", "deskForm");


    } catch (error) {
      mostrarError(`Error añadiendo mesa: ${error.message}`, "deskForm");
    }
  });
});


//Funcion de modal exito o error
function mostrarExito(mensaje, formId = null) {
  const modal = document.getElementById("modal-exito");
  modal.querySelector("p").innerText = mensaje;
  modal.classList.add("mostrar");

  if (formId) {
    const form = document.getElementById(formId);
    if (form) form.reset();
  }
}

function mostrarError(mensaje, formId = null) {
  const modal = document.getElementById("modal-error");
  modal.querySelector("p").innerText = mensaje;
  modal.classList.add("mostrar");

  if (formId) {
    const form = document.getElementById(formId);
    if (form) form.reset();
  }
}

function cerrarModal(id) {
  const modal = document.getElementById(id);
  modal.classList.remove("mostrar");
  location.reload();
}

async function cargarEntregasEnConsola() {
  const consoleElement = document.getElementById('console');

  try {
    const response = await fetch(`${API_URL}/entregas/vista`);

    if (!response.ok) {
      throw new Error(`Error al obtener las entregas: ${response.statusText}`);
    }

    const result = await response.json();

    if (!result.data || !Array.isArray(result.data)) {
      throw new Error('El formato de la respuesta no es el esperado.');
    }

    const entregas = result.data;

    if (entregas.length === 0) {
      consoleElement.textContent = 'No hay entregas disponibles.';
      return;
    }

    let formattedText = '';

    entregas.forEach(entrega => {
      const hora = entrega.hora;
      const tipo = entrega.tipo;
      const completado = entrega.completado ? 'Completado' : 'Pendiente';
      const origen = entrega.dni_origen || 'No disponible';
      const destino = entrega.dni_destino || 'No disponible';
      const robotId = entrega.robot_id_robot;

      if (result.success) {
        formattedText += `----------------------------------------\n
          Hora: ${hora}\n
          Tipo: ${tipo}\n
          Estado: ${completado}\n
          Origen: ${origen}\n
          Destino: ${destino}\n
          Robot ID: ${robotId}\n`;

        formattedText += `----------------------------------------\n\n`;
      }
    });

    if (formattedText) {
      consoleElement.textContent = formattedText;
    } else {
      consoleElement.textContent = 'No hay entregas con paquetes o documentos.';
    }

  } catch (error) {
    consoleElement.textContent = `Error cargando entregas: ${error.message}`;
    console.error('Detalles del error:', error);
  }
}

// Mostrar mensajes del robot
// conectar a ros2
function connect(){

    data.ros = new ROSLIB.Ros({
          url: data.rosbridge_address
  })

  data.ros.on("connection", () => {
      data.connected = true
      console.log("Conexion con ROSBridge correcta")
      suscribirMensajesAdmin()
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

function suscribirMensajesAdmin() {
    const listener = new ROSLIB.Topic({
        ros: data.ros,
        name: '/mensajes_admin',
        messageType: 'std_msgs/String'
    });

    listener.subscribe((message) => {
      const consoleElement = document.getElementById('console');
      ahora = new Date()
      consoleElement.textContent = consoleElement.textContent + "[" + ahora.toLocaleTimeString() + "] " + message.data;
    });
}

// Llamar a la función cuando la página esté lista
document.addEventListener('DOMContentLoaded', cargarEntregasEnConsola);
document.addEventListener('DOMContentLoaded', event => {
    data = {
        ros: null,
        rosbridge_address: 'ws://127.0.0.1:9090/',
        connected: false,
    };

    connect();

    data.ros.on("close", () => {
        data.connected = false;
        console.log("Conexion con ROSBridge cerrada. Intentando reconectar en 5 segundos...");
    
        setTimeout(() => {
            connect();
        }, 5000);
    });

});

// Inicializamos las tablas vacías
renderRobots();
renderWorkers();
renderDesks();
