from extensions import db
from flask import Blueprint, request, jsonify
from models.trabajador import Trabajador
import hashlib

iniciarSesion_api = Blueprint('api_login', __name__)

def hash_contraseña(contraseña):
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

    if hash_contraseña(contraseña) != trabajador.contraseña_hash:
        return jsonify({'error': 'Correo o contraseña incorrectos'}), 401

    trabajador.presente = True
    db.session.commit()

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

    trabajador.presente = False
    db.session.commit()

    return jsonify({'message': 'Sesión cerrada'}), 200