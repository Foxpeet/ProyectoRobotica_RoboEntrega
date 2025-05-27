import hashlib
from flask import Blueprint, request, jsonify
from extensions import db
from sqlalchemy.exc import IntegrityError, OperationalError
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
    try:
        db.session.commit()
        return jsonify({'message': 'Trabajador añadido correctamente'}), 201
    except IntegrityError as e:
        db.session.rollback()  # Deshacer cualquier cambio si ocurre un error
        return jsonify({'message': 'Error de integridad, el trabajador ya puede existir o tiene un conflicto de datos.'}), 400
    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error inesperado: {str(e)}'}), 500

@trabajador_api.route('/trabajadores/no_admin', methods=['GET'])
def get_no_admin_trabajadores():
    trabajadores_no_admin = Trabajador.query.filter_by(rol_admin=False).all()
    
    if not trabajadores_no_admin:
        return jsonify({'message': 'No hay trabajadores no admins'}), 404
    
    return jsonify([
        {
            'dni_trabajador': t.dni_trabajador,
            'nombre_trabajador': t.nombre_trabajador,
            'apellido_trabajador': t.apellido_trabajador,
            'presente': t.presente
        }
        for t in trabajadores_no_admin
    ])

@trabajador_api.route('/trabajadores/<string:dni_trabajador>', methods=['DELETE'])
def delete_trabajador(dni_trabajador):
    trabajador = Trabajador.query.filter_by(dni_trabajador=dni_trabajador).first()

    if not trabajador:
        return jsonify({'message': 'Trabajador no encontrado'}), 404

    try:
        db.session.delete(trabajador)
        db.session.commit()
        return jsonify({'message': f'Trabajador {dni_trabajador} eliminado correctamente'}), 200
    except IntegrityError as e:
        db.session.rollback()  # Deshacer cualquier cambio si ocurre un error
        return jsonify({'message': 'Error de integridad al eliminar trabajador.'}), 400
    except OperationalError as e:
        db.session.rollback()  # En caso de un error operacional, revertimos
        return jsonify({'message': f'Error de base de datos: {str(e)}'}), 500
    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error inesperado: {str(e)}'}), 500
    finally:
        db.session.remove()  # Liberamos la sesión para evitar bloqueos futuros


@trabajador_api.route('/trabajadores/<string:dni_trabajador>', methods=['PUT'])
def update_trabajador(dni_trabajador):
    trabajador = Trabajador.query.filter_by(dni_trabajador=dni_trabajador).first()
    
    if not trabajador:
        return jsonify({'message': 'Trabajador no encontrado'}), 404

    data = request.get_json()
    
    try:
        # Actualizar solo campos permitidos
        if 'correo' in data:
            trabajador.correo = data['correo']
        
        if 'contraseña' in data and data['contraseña']:
            # Hashear la nueva contraseña
            trabajador.contraseña_hash = hashlib.sha256(data['contraseña'].encode('utf-8')).hexdigest()

        db.session.commit()
        return jsonify({
            'message': 'Trabajador actualizado correctamente',
            'dni': trabajador.dni_trabajador,
            'correo': trabajador.correo
        }), 200
        
    except IntegrityError as e:
        db.session.rollback()
        return jsonify({
            'message': 'Error de integridad en los datos',
            'error': str(e.orig)
        }), 400
        
    except Exception as e:
        db.session.rollback()
        return jsonify({
            'message': 'Error al actualizar trabajador',
            'error': str(e)
        }), 500