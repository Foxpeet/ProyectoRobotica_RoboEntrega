from flask import Blueprint, request, jsonify
from extensions import db
from models.entregas import Entrega
from datetime import time

entrega_api = Blueprint('api_entrega', __name__)


@entrega_api.route('/entregas/nocompletadas', methods=['GET'])
def get_entregas_no_completadas():
    try:
        # Obtener entregas no completadas ordenadas de más antigua a más nueva
        entregas = Entrega.query.filter_by(completado=False)\
                              .order_by(Entrega.hora.asc())\
                              .all()
        
        entregas_data = [{
            'id_entrega': e.id_entrega,
            'hora': e.hora.isoformat() if e.hora else None,
            'tipo': e.tipo,
            'dni_origen': e.dni_origen,
            'dni_destino': e.dni_destino,  # Corregido: era 'destino' en tu modelo original
            'completado': e.completado,
            'robot_id_robot': e.robot_id_robot,
        } for e in entregas]
        
        return jsonify({
            'success': True,
            'data': entregas_data,
            'count': len(entregas_data),
            'message': 'Entregas no completadas ordenadas por hora (antiguas primero)'
        }), 200
        
    except SQLAlchemyError as e:
        return jsonify({
            'success': False,
            'error': str(e),
            'message': 'Error de base de datos'
        }), 500
    
@entrega_api.route('/entregas/ultima_pendiente', methods=['GET'])
def get_ultima_entrega_pendiente():
    try:
        # Obtener la entrega no completada más reciente
        ultima_entrega = Entrega.query.filter_by(completado=False)\
                                    .order_by(Entrega.hora.desc())\
                                    .first()

        if not ultima_entrega:
            return jsonify({
                'success': True,
                'data': None,
                'message': 'No hay entregas pendientes'
            }), 200

        # Obtener el trabajador destino y su mesa
        trabajador_destino = Trabajador.query.filter_by(
            dni_trabajador=ultima_entrega.dni_destino
        ).first()
        
        if not trabajador_destino:
            return jsonify({
                'success': False,
                'error': 'Trabajador destino no encontrado',
                'message': f'No se encontró el trabajador con DNI {ultima_entrega.dni_destino}'
            }), 404

        mesa_destino = Mesa.query.get(trabajador_destino.mesa_id_mesa)
        
        if not mesa_destino:
            return jsonify({
                'success': False,
                'error': 'Mesa no encontrada',
                'message': f'No se encontró la mesa con ID {trabajador_destino.mesa_id_mesa}'
            }), 404

        # Construir respuesta con los nombres de campos correctos
        response_data = {
            'entrega': {
                'id': ultima_entrega.id_entrega,
                'hora': ultima_entrega.hora.isoformat() if ultima_entrega.hora else None,
                'tipo': ultima_entrega.tipo,
                'robot_id': ultima_entrega.robot_id_robot
            },
            'destino': {
                'dni': ultima_entrega.dni_destino,
                
                'nombre': f"{trabajador_destino.nombre_trabajador} {trabajador_destino.apellido_trabajador}",
                'mesa': {
                    'id': mesa_destino.id_mesa,
                    'ubicacion': {
                        'longitud': float(mesa_destino.longitud_x),
                        'latitud': float(mesa_destino.latitud_y)
                    }
                }
            }
        }

        return jsonify({
            'success': True,
            'data': response_data,
            'message': 'Última entrega pendiente encontrada'
        }), 200

    except SQLAlchemyError as e:
        db.session.rollback()
        return jsonify({
            'success': False,
            'error': 'Database error',
            'message': str(e)
        }), 500

    except Exception as e:
        return jsonify({
            'success': False,
            'error': 'Server error',
            'message': str(e)
        }), 500