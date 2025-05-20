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
            # Puedes añadir campos relacionados si lo necesitas
            # 'paquetes': [p.id_paquete for p in e.paquetes],
            # 'documentos': [d.id_documento for d in e.documentos]
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