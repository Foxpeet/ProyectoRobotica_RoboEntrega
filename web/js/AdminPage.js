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

const consoleLog = document.getElementById("console");

// MODALES
const editWorkerModal = document.getElementById("editWorkerModal");
const closeEditWorkerModal = document.getElementById("closeEditWorkerModal");
const editWorkerForm = document.getElementById("editWorkerForm");

const changeDeskModal = document.getElementById("changeDeskModal");
const closeChangeDeskModal = document.getElementById("closeChangeDeskModal");
const deskSelect = document.getElementById("deskSelect");
const confirmChangeDeskBtn = document.getElementById("confirmChangeDeskBtn");

const API_URL = 'http://127.0.0.1:5000/api';

// --- FUNCIONES ROBOTS ---
function renderRobots() {
  robotTable.innerHTML = "";
  robots.forEach((r) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${r.id}</td>
      <td>${r.model}</td>
      <td>${r.serial}</td>
      <td>
        <button class="delete-button" title="Eliminar robot">
          <img src="img/basura_eliminar_boton.png" alt="Eliminar" />
        </button>
      </td>`;
    // Añadir listener eliminar
    tr.querySelector(".delete-button").addEventListener("click", () => {
      if (confirm(`¿Eliminar robot ID ${r.id} (${r.model})?`)) {
        robots = robots.filter((robot) => robot.id !== r.id);
        log(`Robot ID ${r.id} eliminado.`);
        renderRobots();
        showNotification(`Robot ID ${r.id} eliminado.`);
      }
    });
    robotTable.appendChild(tr);
  });
}

robotForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const model = document.getElementById("robotModel").value.trim();
  const serial = document.getElementById("robotSerial").value.trim();
  if (!model || !serial) {
    alert("Por favor, rellena modelo y número de serie.");
    return;
  }
  robots.push({ id: nextRobotId++, model, serial });
  log(`Robot "${model}" añadido.`);
  robotForm.reset();
  renderRobots();
  showNotification(`Robot "${model}" añadido correctamente.`);
});

// --- FUNCIONES TRABAJADORES ---
async function renderWorkers() {
  try {
    const response = await fetch(`${API_URL}/trabajadores`);
    const trabajadores = await response.json();

    const table = document.getElementById("workerTable");
    table.innerHTML = ""; // Limpiar la tabla antes de insertar
    console.table(trabajadores);

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
    console.error("❌ Error cargando trabajadores:", err);
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
    console.table(trabajadores);
    
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
    console.error("❌ Error cargando trabajadores:", err);
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

    console.log(`✅ Trabajador eliminado.`);
    mostrarExito(`Trabajador eliminado correctamente`, 'success');
    renderWorkers(); // Recargar la lista
  } catch (err) {
    console.error(`❌ Error eliminando trabajador: ${err}`);
    mostrarError(`Error eliminando trabajador: ${err.message}`, 'error');
  }
}



// Event Listener for the form submission
workerForm.addEventListener("submit", async (e) => {
  e.preventDefault();  // Prevent default form submission

  const nuevoTrabajador = {
    dni_trabajador: document.getElementById('workerDNI').value,
    nombre_trabajador: document.getElementById('workerName').value,
    apellido_trabajador: document.getElementById('workerLastName').value,
    correo: document.getElementById('workerEmail').value,
    contraseña: document.getElementById('workerPassword').value,
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

  // Send the data to the backend API
  try {
      const response = await fetch(`${API_URL}/trabajadores`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(nuevoTrabajador)
      });

      if (response.ok) {
          // Assuming renderWorkers is a function that refreshes the list of workers
          renderWorkers();
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

// --- MODAL ASIGNAR CABINA ---
let currentWorkerForDesk = null;

async function openChangeDeskModal(worker) {
  currentWorkerForDesk = worker;
  changeDeskModal.style.display = "flex";
  deskSelect.innerHTML = ""; // Limpiar el select
  
  try {
    // Llamar a la API para obtener las mesas y su estado
    const response = await fetch('/mesas_con_trabajadores');
    if (!response.ok) {
      throw new Error("Error al obtener las mesas.");
    }

    const desks = await response.json(); // Obtener los datos de las mesas

    // Si no hay mesas, mostrar un mensaje
    if (desks.length === 0) {
      const option = document.createElement("option");
      option.textContent = "No hay cabinas disponibles";
      option.disabled = true;
      deskSelect.appendChild(option);
      confirmChangeDeskBtn.disabled = true;
    } else {
      // Recorrer todas las mesas y crear las opciones
      desks.forEach((desk) => {
        const option = document.createElement("option");
        option.value = desk.id_mesa; // Usamos 'id_mesa' como valor
        
        // Crear el texto de la opción
        option.textContent = `ID ${desk.id_mesa} - Lon: ${desk.longitud_x}, Lat: ${desk.latitud_y}`;
        
        // Si la mesa está ocupada, deshabilitar la opción
        if (desk.ocupada) {
          option.disabled = true;
          option.textContent += " (Ocupada)";
        }

        // Añadir la opción al select
        deskSelect.appendChild(option);
      });

      // Habilitar o deshabilitar el botón de confirmación según disponibilidad
      confirmChangeDeskBtn.disabled = !desks.some(desk => !desk.ocupada); // Habilitar solo si hay mesas libres
    }
  } catch (error) {
    console.error("Error:", error);
    alert("Hubo un problema al cargar las mesas.");
  }
}

closeChangeDeskModal.addEventListener("click", () => {
  changeDeskModal.style.display = "none";
  currentWorkerForDesk = null;
});

confirmChangeDeskBtn.addEventListener("click", () => {
  if (!currentWorkerForDesk) return;

  const selectedDeskId = parseInt(deskSelect.value);
  if (isNaN(selectedDeskId)) {
    alert("Selecciona una cabina válida.");
    return;
  }

  // Aquí puedes agregar el código para realizar la asignación del trabajador a la mesa
  // Por ejemplo, puedes hacer una llamada API para asignar el trabajador a la mesa seleccionada
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
      log("Lista de cabinas vacía recibida de la API");
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
    
    log(`✅ ${desks.length} cabinas cargadas desde API`);
    
  } catch (error) {
    deskTable.innerHTML = `
      <tr>
        <td colspan="3" class="error-message">
          Error al cargar cabinas: ${error.message}
        </td>
      </tr>`;
    
    log(`❌ Error al cargar cabinas: ${error}`);
    console.error("Error en renderDesks:", error);
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
      if (typeof log === "function") {
        log("Mesa añadida correctamente");
      }

    } catch (error) {
      mostrarError(`Error añadiendo mesa: ${error.message}`, "deskForm");

      if (typeof log === "function") {
        log(`❌ Error añadiendo mesa: ${error}`);
      }
    }
  });
});


//Funcion de modal exito o error
function mostrarExito(mensaje, formId = null) {
  console.log("mostrarExito llamada con mensaje:", mensaje);
  const modal = document.getElementById("modal-exito");
  modal.querySelector("p").innerText = mensaje;
  modal.classList.add("mostrar");

  if (formId) {
    const form = document.getElementById(formId);
    if (form) form.reset();
  }
}

function mostrarError(mensaje, formId = null) {
  console.log("mostrarError llamada con mensaje:", mensaje);
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



// Función para loguear en la consola visual
        function log(message) {
            const consoleElem = document.getElementById('console');
            consoleElem.innerHTML += `${new Date().toLocaleTimeString()}: ${message}\n`;
            consoleElem.scrollTop = consoleElem.scrollHeight;
        }

// Inicializamos las tablas vacías
renderRobots();
renderWorkers();
renderDesks();
