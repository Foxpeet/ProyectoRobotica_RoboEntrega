document.addEventListener('DOMContentLoaded', () => {
    // Verificar si ya hay sesión iniciada
    if (sessionStorage.getItem('usuario')) {
        window.location.href = 'mapa.html';
        return;
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
                sessionStorage.setItem('usuario', JSON.stringify(data));
                window.location.href = 'mapa.html';
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
});
