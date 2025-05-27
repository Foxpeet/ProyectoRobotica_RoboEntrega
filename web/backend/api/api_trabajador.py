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

    # Verificar si el trabajador con el mismo DNI ya existe
    trabajador_existente = Trabajador.query.filter_by(dni_trabajador=data['dni_trabajador']).first()
    if trabajador_existente:
        return jsonify({'message': f'El DNI {data["dni_trabajador"]} ya está registrado como {trabajador_existente.nombre_trabajador} {trabajador_existente.apellido_trabajador}.'}), 400
    
    # Verificar si el trabajador con el mismo correo electrónico ya existe
    trabajador_correo_existente = Trabajador.query.filter_by(correo=data['correo']).first()
    if trabajador_correo_existente:
        return jsonify({'message': f'El correo electrónico {data["correo"]} ya está registrado por {trabajador_correo_existente.nombre_trabajador} {trabajador_correo_existente.apellido_trabajador}.'}), 400

    # Encriptar la contraseña
    password_raw = data['contraseña']
    password_hash = hashlib.sha256(password_raw.encode('utf-8')).hexdigest()

    # Crear el nuevo trabajador
    nuevo = Trabajador(
        dni_trabajador=data['dni_trabajador'],
        nombre_trabajador=data['nombre_trabajador'],
        apellido_trabajador=data['apellido_trabajador'],
        correo=data['correo'],
        contraseña_hash=password_hash,
        presente=data.get('presente', False),
        rol_admin=data.get('rol_admin', False),
        mesa_id_mesa = 0
    )

    db.session.add(nuevo)
    try:
        db.session.commit()
        return jsonify({'message': 'Trabajador añadido correctamente'}), 201
    except IntegrityError as e:
        db.session.rollback()  # Deshacer cualquier cambio si ocurre un error
        if "duplicate key value violates unique constraint" in str(e.orig):
            # Si la excepción es de clave duplicada, dar más detalles
            return jsonify({'message': 'Error de integridad: ya existe un trabajador con ese DNI o correo electrónico.'}), 400
        return jsonify({'message': 'Error de integridad, posible conflicto de datos.'}), 400
    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error inesperado: {str(e)}. Por favor, intente nuevamente.'}), 500

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