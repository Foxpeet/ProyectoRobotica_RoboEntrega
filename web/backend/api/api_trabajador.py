import hashlib
from flask import Blueprint, request, jsonify
from extensions import db
from models.trabajador import Trabajador

trabajador_api = Blueprint('api_trabajador', __name__)

@trabajador_api.route('/trabajadores', methods=['GET'])
def get_trabajadores():
    trabajadores = Trabajador.query.all()
    return jsonify([
        {
            'dni_trabajador': t.dni_trabajador,
            'nombre_trabajador': t.nombre_trabajador,
            'apellido_trabajador': t.apellido_trabajador,
            'correo': t.correo,
            'presente': t.presente,
            'rol_admin': t.rol_admin,
            'mesa_id_mesa': t.mesa_id_mesa
        }
        for t in trabajadores
    ])

@trabajador_api.route('/trabajadores', methods=['POST'])
def create_trabajador():
    data = request.get_json()

    # Hashear la contraseña con SHA-256
    password_raw = data['contraseña_hash']
    password_hash = hashlib.sha256(password_raw.encode('utf-8')).hexdigest()
    nuevo = Trabajador(
        dni_trabajador=data['dni_trabajador'],
        nombre_trabajador=data['nombre_trabajador'],
        apellido_trabajador=data['apellido_trabajador'],
        correo=data['correo'],
        contraseña_hash=password_hash,
        presente=data.get('presente', False),
        rol_admin=data.get('rol_admin', False),
        mesa_id_mesa=data['mesa_id_mesa']
    )
    db.session.add(nuevo)
    db.session.commit()

    return jsonify({'message': 'Trabajador añadido correctamente'}), 201
