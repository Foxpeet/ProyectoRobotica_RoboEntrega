from extensions import db  # Importamos db para manejar la sesión
from flask import Blueprint, request, jsonify
from models.trabajador import Trabajador
import hashlib

iniciarSesion_api = Blueprint('api_login', __name__)

def hash_contraseña(contraseña):
    # Hashea la contraseña usando SHA-256
    hash_obj = hashlib.sha256(contraseña.encode('utf-8'))
    return hash_obj.hexdigest()

@iniciarSesion_api.route('/login', methods=['POST'])
def login():
    data = request.get_json()
    correo = data.get('correo')
    contraseña = data.get('contraseña')

    trabajador = Trabajador.query.filter_by(correo=correo).first()

    if not trabajador:
        return jsonify({'error': 'Correo o contraseña incorrectos'}), 401

    # Comparar la contraseña enviada (hash) con la guardada en la base de datos
    if hash_contraseña(contraseña) != trabajador.contraseña_hash:
        return jsonify({'error': 'Correo o contraseña incorrectos'}), 401

    # Actualizar el estado "presente" a True
    trabajador.presente = True
    db.session.commit()

    # Retornar los datos del trabajador si la autenticación es correcta
    return jsonify({
        'dni_trabajador': trabajador.dni_trabajador,
        'nombre_trabajador': trabajador.nombre_trabajador,
        'apellido_trabajador': trabajador.apellido_trabajador,
        'correo': trabajador.correo,
        'presente': trabajador.presente,
        'rol_admin': trabajador.rol_admin,
        'mesa_id_mesa': trabajador.mesa_id_mesa
    }), 200

@iniciarSesion_api.route('/logout', methods=['POST'])
def logout():
    data = request.get_json()
    correo = data.get('correo')

    trabajador = Trabajador.query.filter_by(correo=correo).first()

    if not trabajador:
        return jsonify({'error': 'Trabajador no encontrado'}), 404

    # Solo actualizar el estado "presente" a False
    trabajador.presente = False
    db.session.commit()

    # Respuesta mínima de confirmación
    return jsonify({'message': 'Sesión cerrada'}), 200