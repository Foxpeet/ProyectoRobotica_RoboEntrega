from flask import Blueprint, request, jsonify
from extensions import db
from models.entregas import Entrega

entrega_api = Blueprint('entrega_api', __name__)

@entrega_api.route('/api/entregas', methods=['POST'])
def create_entrega():
    data = request.get_json()

    # Verificar que los campos necesarios estén presentes en la solicitud
    if not all(key in data for key in ('hora', 'tipo', 'dni_destino', 'robot_id_robot', 'trabajador_dni_trabajador')):
        return jsonify({'error': 'Faltan campos obligatorios'}), 400

    try:
        # Crear una nueva entrega
        nueva_entrega = Entrega(
            hora=data['hora'],
            tipo=data['tipo'],
            dni_origen=data.get('dni_origen', None),  # dni_origen es opcional
            dni_destino=data['dni_destino'],
            robot_id_robot=data['robot_id_robot'],
            trabajador_dni_trabajador=data['trabajador_dni_trabajador']
        )

        # Agregar la entrega a la sesión de la base de datos
        db.session.add(nueva_entrega)
        db.session.commit()

        return jsonify({
            'message': 'Entrega creada exitosamente',
            'id_entrega': nueva_entrega.id_entrega
        }), 201

    except Exception as e:
        # Manejar errores
        db.session.rollback()
        return jsonify({'error': str(e)}), 500
