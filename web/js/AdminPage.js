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

// Función para eliminar trabajador (separada para mejor organización)
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


workerForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const dni = document.getElementById("workerDNI").value.trim();
  const name = document.getElementById("workerName").value.trim();
  const lastName = document.getElementById("workerLastName").value.trim();
  const email = document.getElementById("workerEmail").value.trim();
  const password = document.getElementById("workerPassword").value;
  if (!dni || !name || !lastName || !email || !password) {
    alert("Por favor, rellena todos los campos.");
    return;
  }
  workers.push({
    id: nextWorkerId++,
    dni,
    name,
    lastName,
    email,
    password,
    assignedDeskId: null,
  });
  log(`Trabajador "${name} ${lastName}" añadido.`);
  workerForm.reset();
  renderWorkers();
  showNotification(`Trabajador "${name} ${lastName}" añadido correctamente.`);
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

function openChangeDeskModal(worker) {
  currentWorkerForDesk = worker;
  changeDeskModal.style.display = "flex";
  deskSelect.innerHTML = "";
  if (desks.length === 0) {
    const option = document.createElement("option");
    option.textContent = "No hay cabinas disponibles";
    option.disabled = true;
    deskSelect.appendChild(option);
    confirmChangeDeskBtn.disabled = true;
  } else {
    desks.forEach((desk) => {
      const option = document.createElement("option");
      option.value = desk.id;
      option.textContent = `ID ${desk.id} - Lon: ${desk.longitude}, Lat: ${desk.latitude}`;
      deskSelect.appendChild(option);
    });
    confirmChangeDeskBtn.disabled = false;
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
  // Asignar cabina
  const workerIndex = workers.findIndex((w) => w.id === currentWorkerForDesk.id);
  if (workerIndex >= 0) {
    workers[workerIndex].assignedDeskId = selectedDeskId;
    log(`Trabajador "${workers[workerIndex].name}" asignado a cabina ID ${selectedDeskId}.`);
    changeDeskModal.style.display = "none";
    currentWorkerForDesk = null;
    renderWorkers();
    showNotification(`Trabajador "${workers[workerIndex].name}" asignado a cabina ID ${selectedDeskId}.`);
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
