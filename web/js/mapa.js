document.addEventListener('DOMContentLoaded', () => {
    const usuario = sessionStorage.getItem('usuario');

    if (!usuario) {
        window.location.href = 'iniciarSesion.html';
        return;
    }
    const datosUsuario = JSON.parse(usuario);

    const nombre = datosUsuario.nombre_trabajador;
    const apellido = datosUsuario.apellido_trabajador;

    const bienvenida = document.getElementById('bienvenida_mapa');
    if (bienvenida) {
        bienvenida.textContent = `¡Bienvenido, ${nombre} ${apellido}!`;
    } else {
        console.error('No se encontró el elemento de bienvenida en el DOM');
    }

    const dniLogueado = datosUsuario.dni_trabajador;

    const trabajadoresContainer = document.getElementById('trabajadores-lista');
    trabajadoresContainer.style.display = 'grid';
    trabajadoresContainer.style.gridTemplateColumns = 'repeat(auto-fill, minmax(200px, 1fr))';

    const modal = document.getElementById("modal"); // Modal de confirmación
    const btnSí = document.getElementById("btn-sí");
    const btnNo = document.getElementById("btn-no");

    const modalConfirmacion = document.getElementById('modal-entrega-enviada');
    const closeModalEntrega = document.getElementById('close-modal-entrega');

    // Iniciar ambos modales como ocultos
    modal.style.display = "none";
    modalConfirmacion.style.display = "none";

    // Realizar la solicitud a la API con el DNI del trabajador logueado
    fetch(`http://127.0.0.1:5000/api/trabajadores/no_admin?dni_logueado=${dniLogueado}`)
        .then(response => response.json())
        .then(data => {
            trabajadoresContainer.innerHTML = '';
            if (data.length === 0) {
                trabajadoresContainer.innerHTML = 'No hay más trabajadores disponibles.';
                return;
            }

            data.forEach(trabajador => {
                const button = document.createElement('button');
                button.textContent = `${trabajador.nombre_trabajador} ${trabajador.apellido_trabajador}`;
                button.classList.add('trabajador-button');

                if (!trabajador.presente) {
                    button.disabled = true;
                    button.classList.add('disabled');
                } else {
                    button.addEventListener('click', () => {
                        document.getElementById("nombre-trabajador-modal").textContent = trabajador.nombre_trabajador;
                        document.getElementById("apellido-trabajador-modal").textContent = trabajador.apellido_trabajador;
                        modal.style.display = "block";

                        btnSí.addEventListener('click', () => {
                            const trabajadorNombre = trabajador.nombre_trabajador;
                            const trabajadorApellido = trabajador.apellido_trabajador;
                            const dniDestino = trabajador.dni_trabajador;

                            const usuario = sessionStorage.getItem('usuario');
                            const datosUsuario = JSON.parse(usuario);
                            const dniOrigen = datosUsuario.dni_trabajador;

                            const entregaData = {
                                id_entrega: `entrega_${Date.now()}`,  // Generar un ID único para la entrega (puedes cambiar esto)
                                dni_origen: dniOrigen,
                                dni_destino: dniDestino,
                                robot_id_robot: 1,  // Este es un valor fijo que tendrías que modificar según tu lógica (puedes asignarlo dinámicamente si es necesario)
                                documentos: ["Mensaje por defecto"]  // Asumiendo que estás enviando documentos, lo llenamos vacío por ahora
                            };
                            console.log ( entregaData);
                            // Hacer la llamada a la API para crear la entrega
                            fetch('http://127.0.0.1:5000/api/entregas', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/json',
                                },
                                body: JSON.stringify(entregaData),
                            })
                            .then(response => response.json())
                            .then(data => {
                                if (data.success) {
                                    // Mostrar el modal de "Entrega Enviada"
                                    modalConfirmacion.style.display = "block";

                                    // Cerrar el modal de confirmación
                                    modal.style.display = "none";
                                }
                            })
                            .catch(error => {
                                console.error('Error al crear la entrega:', error);
                                alert('Hubo un error al procesar la entrega. Inténtalo de nuevo.');
                            });
                        });

                        // Lógica para manejar el botón "No"
                        btnNo.addEventListener('click', () => {
                            // Solo cerrar el modal de confirmación sin hacer nada
                            modal.style.display = "none";
                        });
                    });
                }

                // Añadir el botón al contenedor
                trabajadoresContainer.appendChild(button);
            });
        })
        .catch(error => {
            console.error('Error al cargar la lista de trabajadores:', error);
            trabajadoresContainer.innerHTML = 'No se pudo cargar la lista de trabajadores. Intenta nuevamente más tarde.';
        });

    // Cerrar el modal de "Entrega Enviada" con la X
    closeModalEntrega.addEventListener('click', () => {
        modalConfirmacion.style.display = "none";  // Cerrar el modal de éxito
    });

    // Gestionar el cierre de sesión
    const logoutLink = document.getElementById('logout-link');
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
