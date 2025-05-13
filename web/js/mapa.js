document.addEventListener('DOMContentLoaded', () => {
    // Verificar si el usuario está logueado
    const usuario = sessionStorage.getItem('usuario');

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
    const bienvenida = document.getElementById('bienvenida_mapa');
    bienvenida.textContent = `¡Bienvenido, ${nombre} ${apellido}!`;

    // Obtener la lista de trabajadores no admin
    const trabajadoresContainer = document.getElementById('trabajadores-lista');
    trabajadoresContainer.style.display = 'grid'; // Disposición en grid para mejor organización
    trabajadoresContainer.style.gridTemplateColumns = 'repeat(auto-fill, minmax(200px, 1fr))'; // Distribuir los botones en varias columnas

    fetch('http://127.0.0.1:5000/api/trabajadores/no_admin')
        .then(response => {
            if (!response.ok) {
                throw new Error('Error al cargar la lista de trabajadores.');
            }
            return response.json();
        })
        .then(data => {
            if (data.message) {
                trabajadoresContainer.innerHTML = data.message; // Mostrar mensaje si no hay trabajadores
            } else {
                data.forEach(trabajador => {
                    // Crear un botón para cada trabajador
                    const button = document.createElement('button');
                    button.textContent = `${trabajador.nombre_trabajador} ${trabajador.apellido_trabajador}`;
                    button.classList.add('trabajador-button');
                    
                    // Deshabilitar el botón si el trabajador no está presente
                    if (!trabajador.presente) {
                        button.disabled = true;
                        button.classList.add('disabled'); // Se puede agregar una clase para deshabilitarlo visualmente
                    }
                    
                    // Añadir el botón al contenedor
                    trabajadoresContainer.appendChild(button);
                });
            }
        })
        .catch(error => {
            console.error('Error al cargar la lista de trabajadores:', error);
            trabajadoresContainer.innerHTML = 'No se pudo cargar la lista de trabajadores. Intenta nuevamente más tarde.';
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
