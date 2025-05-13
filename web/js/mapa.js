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

    // Gestionar el cierre de sesión
    const logoutLink = document.getElementById('logout-link');
    logoutLink.addEventListener('click', (e) => {
        // Eliminar el usuario de sessionStorage
        sessionStorage.removeItem('usuario');
        
        // Redirigir al inicio de sesión
        window.location.href = 'iniciarSesion.html';
    });
});
