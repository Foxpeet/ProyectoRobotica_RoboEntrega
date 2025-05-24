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

// --- FUNCIONES DE NOTIFICACIÓN ---
function showNotification(message, isError = false) {
  // Crear el div si no existe
  let notif = document.getElementById("notification");
  if (!notif) {
    notif = document.createElement("div");
    notif.id = "notification";
    notif.style.position = "fixed";
    notif.style.top = "20px";
    notif.style.right = "20px";
    notif.style.padding = "15px 25px";
    notif.style.backgroundColor = isError ? "#e74c3c" : "#2ecc71";
    notif.style.color = "white";
    notif.style.borderRadius = "5px";
    notif.style.boxShadow = "0 2px 10px rgba(0,0,0,0.3)";
    notif.style.zIndex = "9999";
    notif.style.fontSize = "16px";
    notif.style.fontWeight = "bold";
    notif.style.opacity = "0";
    notif.style.transition = "opacity 0.3s ease";
    document.body.appendChild(notif);
  }
  notif.textContent = message;
  notif.style.backgroundColor = isError ? "#e74c3c" : "#2ecc71";
  notif.style.opacity = "1";

  // Desaparecer después de 3 segundos
  setTimeout(() => {
    notif.style.opacity = "0";
  }, 3000);
}

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
function renderWorkers() {
  workerTable.innerHTML = "";
  workers.forEach((w) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${w.id}</td>
      <td>${w.dni}</td>
      <td>${w.name}</td>
      <td>${w.lastName}</td>
      <td>${w.email}</td>
      <td>${"*".repeat(w.password.length)}</td>
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
    // Editar trabajador
    tr.querySelector(".edit-button").addEventListener("click", () => {
      openEditWorkerModal(w);
    });
    // Eliminar trabajador
    tr.querySelector(".delete-button").addEventListener("click", () => {
      if (confirm(`¿Eliminar trabajador ${w.name} ${w.lastName}?`)) {
        workers = workers.filter((worker) => worker.id !== w.id);
        log(`Trabajador ${w.name} eliminado.`);
        renderWorkers();
        showNotification(`Trabajador ${w.name} eliminado.`);
      }
    });
    // Asignar cabina
    tr.querySelector(".change-button").addEventListener("click", () => {
      openChangeDeskModal(w);
    });

    workerTable.appendChild(tr);
  });
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
function openEditWorkerModal(worker) {
  editWorkerModal.style.display = "flex";
  document.getElementById("editWorkerId").value = worker.id;
  document.getElementById("editWorkerDNI").value = worker.dni;
  document.getElementById("editWorkerName").value = worker.name;
  document.getElementById("editWorkerLastName").value = worker.lastName;
  document.getElementById("editWorkerEmail").value = worker.email;
  document.getElementById("editWorkerPassword").value = worker.password;
}
closeEditWorkerModal.addEventListener("click", () => {
  editWorkerModal.style.display = "none";
});
editWorkerForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const id = parseInt(document.getElementById("editWorkerId").value);
  const dni = document.getElementById("editWorkerDNI").value.trim();
  const name = document.getElementById("editWorkerName").value.trim();
  const lastName = document.getElementById("editWorkerLastName").value.trim();
  const email = document.getElementById("editWorkerEmail").value.trim();
  const password = document.getElementById("editWorkerPassword").value;
  if (!dni || !name || !lastName || !email || !password) {
    alert("Por favor, rellena todos los campos.");
    return;
  }
  const workerIndex = workers.findIndex((w) => w.id === id);
  if (workerIndex >= 0) {
    workers[workerIndex].dni = dni;
    workers[workerIndex].name = name;
    workers[workerIndex].lastName = lastName;
    workers[workerIndex].email = email;
    workers[workerIndex].password = password;
    log(`Trabajador "${name} ${lastName}" modificado.`);
    renderWorkers();
    editWorkerModal.style.display = "none";
    showNotification(`Trabajador "${name} ${lastName}" modificado.`);
  }
});

// --- MODAL ASIGNAR CABINA ---
let currentWorkerForDesk = null;

function openChangeDeskModal(worker) {
  currentWorkerForDesk = worker;
  changeDeskModal.style.display = "flex";
  deskSelect.innerHTML = ""; // Limpiar opciones
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

// --- FUNCIONES CABINAS ---
function renderDesks() {
  deskTable.innerHTML = "";
  desks.forEach((desk) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${desk.id}</td>
      <td>${desk.longitude}</td>
      <td>${desk.latitude}</td>
    `;
    deskTable.appendChild(tr);
  });
}

deskForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const longitude = parseFloat(document.getElementById("deskLongitude").value);
  const latitude = parseFloat(document.getElementById("deskLatitude").value);
  if (isNaN(longitude) || isNaN(latitude)) {
    alert("Por favor, introduce valores válidos para longitud y latitud.");
    return;
  }
  desks.push({ id: nextDeskId++, longitude, latitude });
  log(`Cabina añadida: Longitud ${longitude}, Latitud ${latitude}`);
  deskForm.reset();
  renderDesks();
  showNotification(`Cabina añadida: Longitud ${longitude}, Latitud ${latitude}`);
});

// --- Consola de actividad ---
function log(text) {
  const date = new Date().toLocaleTimeString();
  consoleLog.textContent += `[${date}] ${text}\n`;
  consoleLog.scrollTop = consoleLog.scrollHeight;
}

// Inicializamos las tablas vacías
renderRobots();
renderWorkers();
renderDesks();
