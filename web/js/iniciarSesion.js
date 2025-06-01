document.addEventListener('DOMContentLoaded', () => {
    // Verificar si ya hay sesión iniciada
    const usuario = sessionStorage.getItem('usuario');
    if (usuario) {
        const usuarioData = JSON.parse(usuario); // Convertir la cadena a objeto
        if (usuarioData.rol_admin === false) {
            window.location.href = 'mapa.html';
        } else if (usuarioData.rol_admin === true) {
            window.location.href = 'AdminPage.html';
        }
    }

    const form = document.getElementById('login-form');
    const errorMsg = document.getElementById('login-error');
    const loginButton = document.getElementById('login-button');
    const emailInput = document.getElementById('login-email');
    const passwordInput = document.getElementById('login-password');

    // Limpiar el mensaje de error al escribir
    emailInput.addEventListener('input', () => {
        errorMsg.style.display = 'none';
    });

    passwordInput.addEventListener('input', () => {
        errorMsg.style.display = 'none';
    });

    form.addEventListener('submit', async (e) => {
        e.preventDefault();

        // Obtener los valores de correo y contraseña
        const correo = emailInput.value.trim();
        const contraseña = passwordInput.value.trim();

        // Validar si los campos están vacíos
        if (!correo || !contraseña) {
            errorMsg.textContent = 'Por favor ingrese correo y contraseña';
            errorMsg.style.display = 'block';
            return;
        }

        // Deshabilitar el botón mientras se procesa la solicitud
        loginButton.disabled = true;

        try {
            const response = await fetch('http://127.0.0.1:5000/api/login', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ correo, contraseña })
            });

            if (response.ok) {
                const data = await response.json();
                sessionStorage.setItem('usuario', JSON.stringify(data)); // Guardar el usuario en sessionStorage
                
                if (data.rol_admin === false) {
                    window.location.href = 'mapa.html';
                } else if (data.rol_admin === true) {
                    window.location.href = 'AdminPage.html';
                }
            } else if (response.status === 401) {
                errorMsg.textContent = 'Correo o contraseña incorrectos';
                errorMsg.style.display = 'block';
            } else {
                errorMsg.textContent = 'Error en el servidor. Intenta más tarde';
                errorMsg.style.display = 'block';
            }
        } catch (error) {
            console.error('Error al iniciar sesión:', error);
            errorMsg.textContent = 'Error al conectar con el servidor';
            errorMsg.style.display = 'block';
        } finally {
            // Habilitar el botón de nuevo después de procesar la solicitud
            loginButton.disabled = false;   
        }
    });
    console.log(sessionStorage.getItem('usuario'));

});
