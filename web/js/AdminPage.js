// Variables globales para almacenar datos en memoria (simulando DB)
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
  // Verificar si el usuario está logueado
   usuario = sessionStorage.getItem('usuario');

  if (!usuario) {
      // Si no hay sesión activa, redirigir al inicio de sesión
      window.location.href = 'iniciarSesion.html';
      return;
  }

  // Parsear el objeto usuario desde sessionStorage
  const datosUsuario = JSON.parse(usuario);

  // Obtener el nombre y apellido del usuario
  const nombre = datosUsuario.nombre_trabajador;
  const apellido = datosUsuario.apellido_trabajador;

  // Actualizar el título de bienvenida con el nombre y apellido
  const bienvenida = document.getElementById('bievenida_admin');
  bienvenida.textContent = `¡Bienvenido, ${nombre}!`;


// Gestionar el cierre de sesión
const logoutLink = document.getElementById('header-logout-link');
logoutLink.addEventListener('click', async (e) => {
    e.preventDefault(); // Prevenir el comportamiento por defecto del enlace
    
    try {
        // Llamar al endpoint de logout
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
            // Eliminar el usuario de sessionStorage
            sessionStorage.removeItem('usuario');
            
            // Redirigir al inicio de sesión
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

    // Añadir listener eliminar
    tr.querySelector(".delete-button").addEventListener("click", () => {
      openDeleteRobotModal(r); 
    });

    robotTable.appendChild(tr);
  });
}

// Función para abrir el modal de confirmación
function openDeleteRobotModal(robot) {
  const modal = document.getElementById("deleteRobotModal");
  const message = document.getElementById("deleteRobotMessage");
  const confirmBtn = document.getElementById("confirmDeleteRobotBtn");
  const cancelBtn = document.getElementById("cancelDeleteRobotBtn");

  // Establecer el mensaje con información del robot
  message.textContent = `¿Eliminar robot ${robot.modelo_robot} ${robot.numero_serie_robot}?`;

  // Mostrar el modal
  modal.style.display = "flex";

  // Confirmar eliminación
  confirmBtn.onclick = async () => {
    try {
      const deleteResponse = await fetch(`${API_URL}/robots/${robot.id_robot}`, {
        method: 'DELETE',
      });

      if (deleteResponse.ok) {
        mostrarExito(`Robot ID ${robot.id_robot} eliminado.`);
        renderRobots(); // Actualizar la lista de robots
      } else {
        mostrarError("Error al eliminar el robot.");
      }

      // Cerrar el modal después de la eliminación
      closeDeleteRobotModal();
    } catch (error) {
      mostrarError("Hubo un error al eliminar el robot.");
    }
  };

  // Cancelar eliminación
  cancelBtn.onclick = () => {
    closeDeleteRobotModal();
  };

  // Cerrar el modal cuando se haga clic en el botón de cerrar
  document.getElementById("closeDeleteRobotModal").onclick = closeDeleteRobotModal;
}

// Función para cerrar el modal
function closeDeleteRobotModal() {
  const modal = document.getElementById("deleteRobotModal");
  modal.style.display = "none";
}


// Añadir nuevo robot
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
    table.innerHTML = ""; // Limpiar la tabla antes de insertar

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
    table.innerHTML = ""; // Limpiar la tabla antes de insertar
    
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
    renderWorkers(); // Recargar la lista
  } catch (err) {
    mostrarError(`Error eliminando trabajador: ${err.message}`, 'error');
  }
}

async function loadDesks() {
  const deskSelect = document.getElementById("deskSelect");

  try {
    // Llamar a la API para obtener las mesas y su estado
    const response = await fetch(`${API_URL}/mesas_con_trabajadores`);
    
    // Revisar si la respuesta es correcta
    if (!response.ok) {
      const errorDetails = await response.text();  // Capturar el mensaje de error
      throw new Error(`Error al obtener las mesas: ${response.status} - ${errorDetails}`);
    }

    const desks = await response.json(); // Obtener los datos de las mesas

    // Limpiar las opciones del select antes de agregar las nuevas (excepto el placeholder)
    deskSelect.innerHTML = '';  // Eliminar todas las opciones
    const placeholder = document.createElement('option');
    placeholder.value = '';
    placeholder.disabled = true;
    placeholder.selected = true;
    placeholder.textContent = "Elige una mesa";
    deskSelect.appendChild(placeholder); // Agregar el placeholder al principio

    // Si no hay mesas, mostrar un mensaje
    if (desks.length === 0) {
      const option = document.createElement("option");
      option.textContent = "No hay cabinas disponibles";
      option.disabled = true;
      deskSelect.appendChild(option);
    } else {

      // Recorrer todas las mesas y crear las opciones
      desks.forEach((desk) => {
        const option = document.createElement("option");
        option.value = desk.id_mesa; // Usamos 'id_mesa' como valor
        
        // Crear el texto de la opción
        option.textContent = `Mesa - ${desk.id_mesa}`;
        
        // Si la mesa está ocupada, deshabilitar la opción
        if (desk.ocupada) {
          option.disabled = true;
          option.textContent += " (Ocupada)";
        }

        // Añadir la opción al select
        deskSelect.appendChild(option);
      });
    }
  } catch (error) {
    mostrarError("Hubo un problema al cargar las mesas. Ver consola para más detalles.");
  }
}

// Cargar las mesas cuando se cargue el documento
document.addEventListener('DOMContentLoaded', loadDesks);

workerForm.addEventListener("submit", async (e) => {
  e.preventDefault();  // Prevent default form submission
  const nuevoTrabajador = {
    dni_trabajador: document.getElementById('workerDNI').value,
    nombre_trabajador: document.getElementById('workerName').value,
    apellido_trabajador: document.getElementById('workerLastName').value,
    correo: document.getElementById('workerEmail').value,
    contraseña: document.getElementById('workerPassword').value,
    id_mesa: document.getElementById('deskSelect').value  // Aquí se toma el valor de la mesa seleccionada
  };

  // Basic validation for empty fields
  if (!nuevoTrabajador.dni_trabajador || 
      !nuevoTrabajador.nombre_trabajador || 
      !nuevoTrabajador.apellido_trabajador || 
      !nuevoTrabajador.correo || 
      !nuevoTrabajador.contraseña) {
      mostrarError("Por favor, rellena todos los campos obligatorios");
      return;
  }

  // Validate DNI format (8 digits + 1 letter)
  if (!/^[0-9]{8}[A-Za-z]$/.test(nuevoTrabajador.dni_trabajador)) {
      mostrarError("El DNI debe tener 8 números y 1 letra");
      return;
  }

  // Validate email format
  if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(nuevoTrabajador.correo)) {
      mostrarError("Por favor, introduce un email válido");
      return;
  }

  // Validate that a desk has been selected
  if (!nuevoTrabajador.id_mesa) {
      mostrarError("Por favor, selecciona una mesa para el trabajador");
      return;
  }

  // Send the data to the backend API
  try {
      const response = await fetch(`${API_URL}/trabajadores`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(nuevoTrabajador)
      });

      if (response.ok) {
          renderWorkers();  // Refresh the worker list after adding
          e.target.reset();  // Reset the form fields
          mostrarExito('Trabajador añadido correctamente');
      } else {
          const responseData = await response.json();
          // Custom error handling for specific error messages
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
  currentWorkerForDesk = worker;  // Guardamos el trabajador actual para cambiar de mesa
  const changeDeskName = document.getElementById("changeDeskName");
  changeDeskName.textContent = `Cambiar Cabina para ${worker.nombre_trabajador} ${worker.apellido_trabajador}`;  changeDeskModal.style.display = "flex";  // Abrir el modal
  deskChange.innerHTML = "";  // Limpiar las opciones previas del select
  const placeholder = document.createElement('option');
  placeholder.value = '';
  placeholder.disabled = true;
  placeholder.selected = true;
  placeholder.textContent = "Elige nueva mesa";
  deskChange.appendChild(placeholder); // Agregar el placeholder al principio

  try {
    // Llamar a la API para obtener las mesas y su estado
    const response = await fetch(`${API_URL}/mesas_con_trabajadores`);
    if (!response.ok) {
      throw new Error("Error al obtener las mesas.");
    }

    const desks = await response.json(); // Obtener los datos de las mesas

    // Si no hay mesas, mostrar un mensaje y deshabilitar el botón de cambio
    if (desks.length === 0) {
      const option = document.createElement("option");
      option.textContent = "No hay cabinas disponibles";
      option.disabled = true;
      deskChange.appendChild(option);
      confirmChangeDeskBtn.disabled = true;  // Deshabilitar el botón si no hay mesas
    } else {
      let hasAvailableDesk = false;  // Flag para verificar si hay mesas libres

      // Recorrer todas las mesas y crear las opciones
      desks.forEach((desk) => {
        const option = document.createElement("option");
        option.value = desk.id_mesa;  // Asignamos el id_mesa como el valor de la opción

        // Crear el texto de la opción
        option.textContent = `Mesa - ${desk.id_mesa}`;

        // Si la mesa está ocupada, deshabilitar la opción
        if (desk.ocupada) {
          option.disabled = true;
          option.textContent += " (Ocupada)";  // Añadir "(Ocupada)" al texto
        } else {
          hasAvailableDesk = true;  // Hay al menos una mesa libre
        }

        // Añadir la opción al select
        deskChange.appendChild(option);
      });

      // Habilitar o deshabilitar el botón de confirmación según la disponibilidad de mesas libres
      confirmChangeDeskBtn.disabled = !hasAvailableDesk; // Solo habilitar si hay mesas libres
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

  // Verificar si se ha seleccionado una mesa
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
      changeDeskModal.style.display = "none";  // Cerrar el modal después del cambio
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
    
    // 5. Generar el HTML de la tabla
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
  location.reload(); // Recarga la página al cerrar el modal
}


// Función para cargar las entregas en la consola con formato amigable
async function cargarEntregasEnConsola() {
  const consoleElement = document.getElementById('console');

  try {
    const response = await fetch(`${API_URL}/entregas/vista`);

    if (!response.ok) {
      throw new Error(`Error al obtener las entregas: ${response.statusText}`);
    }

    const result = await response.json();

    // Verifica si el campo 'data' existe y es un array
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
      const origen = entrega.dni_origen || 'No disponible';  // Maneja el caso cuando no esté presente
      const destino = entrega.dni_destino || 'No disponible'; // Maneja el caso cuando no esté presente
      const robotId = entrega.robot_id_robot;

      // Verifica si el éxito de la respuesta es true
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
    // Muestra el error completo en la consola
    consoleElement.textContent = `Error cargando entregas: ${error.message}`;

    // Log del error completo para depuración
    console.error('Detalles del error:', error);
  }
}

// Llamar a la función cuando la página esté lista
document.addEventListener('DOMContentLoaded', cargarEntregasEnConsola);

// Inicializamos las tablas vacías
renderRobots();
renderWorkers();
renderDesks();
