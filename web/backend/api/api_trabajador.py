import hashlib
from flask import Blueprint, request, jsonify
from extensions import db
from sqlalchemy.exc import IntegrityError, OperationalError
from models.trabajador import Trabajador
from models.mesa import Mesa



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

    trabajador_existente = Trabajador.query.filter_by(dni_trabajador=data['dni_trabajador']).first()
    if trabajador_existente:
        return jsonify({'message': f'El DNI {data["dni_trabajador"]} ya está registrado como {trabajador_existente.nombre_trabajador} {trabajador_existente.apellido_trabajador}.'}), 400
    
    trabajador_correo_existente = Trabajador.query.filter_by(correo=data['correo']).first()
    if trabajador_correo_existente:
        return jsonify({'message': f'El correo electrónico {data["correo"]} ya está registrado por {trabajador_correo_existente.nombre_trabajador} {trabajador_correo_existente.apellido_trabajador}.'}), 400

    password_raw = data['contraseña']
    password_hash = hashlib.sha256(password_raw.encode('utf-8')).hexdigest()

    nuevo = Trabajador(
        dni_trabajador=data['dni_trabajador'],
        nombre_trabajador=data['nombre_trabajador'],
        apellido_trabajador=data['apellido_trabajador'],
        correo=data['correo'],
        contraseña_hash=password_hash,
        presente=data.get('presente', False),
        rol_admin=data.get('rol_admin', False),
        mesa_id_mesa=data['id_mesa'],
    )

    db.session.add(nuevo)
    try:
        db.session.commit()
        return jsonify({'message': 'Trabajador añadido correctamente'}), 201
    except IntegrityError as e:
        db.session.rollback()
        error_message = str(e.orig)

        if "duplicate key value violates unique constraint" in error_message:
            if 'dni_trabajador' in error_message:
                return jsonify({'message': 'Error de integridad: El DNI ya está registrado.'}), 400
            elif 'correo' in error_message:
                return jsonify({'message': 'Error de integridad: El correo electrónico ya está registrado.'}), 400
            else:
                return jsonify({'message': f'Error de integridad, conflicto con una clave única. Detalles: {error_message}'}), 400
        
        return jsonify({'message': f'Error de integridad: {error_message}'}), 400

    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error inesperado: {str(e)}. Por favor, intente nuevamente.'}), 500


@trabajador_api.route('/trabajadores/no_admin', methods=['GET'])
def get_no_admin_trabajadores():
    try:
        dni_logueado = request.args.get('dni_logueado')
        if not dni_logueado:
            return jsonify({'error': 'El DNI del trabajador logueado es necesario'}), 400

        trabajadores_no_admin = Trabajador.query.filter_by(rol_admin=False).all()

        if not trabajadores_no_admin:
            return jsonify([])

        trabajadores_no_admin = [t for t in trabajadores_no_admin if t.dni_trabajador != dni_logueado]

        return jsonify([{
            'dni_trabajador': t.dni_trabajador,
            'nombre_trabajador': t.nombre_trabajador,
            'apellido_trabajador': t.apellido_trabajador,
            'presente': t.presente
        } for t in trabajadores_no_admin])

    except Exception as e:
        return jsonify({'error': str(e)}), 500


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
        db.session.rollback()
        return jsonify({'message': 'Error de integridad al eliminar trabajador.'}), 400
    except OperationalError as e:
        db.session.rollback()
        return jsonify({'message': f'Error de base de datos: {str(e)}'}), 500
    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error inesperado: {str(e)}'}), 500
    finally:
        db.session.remove()



@trabajador_api.route('/trabajadores/<string:dni_trabajador>', methods=['PUT'])
def update_trabajador(dni_trabajador):
    trabajador = Trabajador.query.filter_by(dni_trabajador=dni_trabajador).first()
    
    if not trabajador:
        return jsonify({'message': 'Trabajador no encontrado'}), 404

    data = request.get_json()
    
    try:
        if 'correo' in data:
            trabajador.correo = data['correo']
        
        if 'contraseña' in data and data['contraseña']:
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
    

@trabajador_api.route('/trabajadores/<dni_trabajador>/cambiar_mesa', methods=['PATCH'])
def cambiar_mesa(dni_trabajador):
    data = request.get_json()
    nuevo_id_mesa = data.get('id_mesa')

    trabajador = Trabajador.query.filter_by(dni_trabajador=dni_trabajador).first()
    if not trabajador:
        return jsonify({'message': 'Trabajador no encontrado'}), 404

    nueva_mesa = Mesa.query.filter_by(id_mesa=nuevo_id_mesa).first()
    if not nueva_mesa:
        return jsonify({'message': 'Mesa no encontrada'}), 404

    mesa_ocupada = Trabajador.query.filter_by(mesa_id_mesa=nuevo_id_mesa).first()
    if mesa_ocupada:
        return jsonify({'message': 'La mesa seleccionada ya está ocupada por otro trabajador'}), 400

    trabajador.mesa_id_mesa = nuevo_id_mesa

    try:
        db.session.commit()
        return jsonify({'message': 'La mesa ha sido cambiada con éxito'}), 200
    except IntegrityError as e:
        db.session.rollback()
        return jsonify({'message': f'Error de integridad: {str(e)}'}), 400
    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error inesperado: {str(e)}'}), 500
