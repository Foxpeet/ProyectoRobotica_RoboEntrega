from flask import Blueprint, request, jsonify
from extensions import db
from models.entregas import Entrega
from models.trabajador import Trabajador
from models.mesa import Mesa

entrega_api = Blueprint('api_entrega', __name__)


@entrega_api.route('/entregas/nocompletadas/ubicaciones', methods=['GET'])
def get_entregas_ubicaciones():
    try:
        # Aliases para joins múltiples con la misma tabla
        TrabajadorOrigen = db.aliased(Trabajador)
        TrabajadorDestino = db.aliased(Trabajador)
        MesaOrigen = db.aliased(Mesa)
        MesaDestino = db.aliased(Mesa)

        entregas = db.session.query(
            Entrega.id_entrega,
            Entrega.hora,
            Entrega.dni_origen,
            Entrega.dni_destino,
            MesaOrigen.longitud_x.label('origen_longitud'),
            MesaOrigen.latitud_y.label('origen_latitud'),
            MesaDestino.longitud_x.label('destino_longitud'),
            MesaDestino.latitud_y.label('destino_latitud')
        ).join(
            TrabajadorDestino, Entrega.dni_destino == TrabajadorDestino.dni_trabajador
        ).join(
            MesaDestino, TrabajadorDestino.mesa_id_mesa == MesaDestino.id_mesa
        ).outerjoin(
            TrabajadorOrigen, Entrega.dni_origen == TrabajadorOrigen.dni_trabajador
        ).outerjoin(
            MesaOrigen, TrabajadorOrigen.mesa_id_mesa == MesaOrigen.id_mesa
        ).filter(
            Entrega.completado == False
        ).order_by(
            Entrega.hora.asc()
        ).all()

        ubicaciones_data = []
        for e in entregas:
            item = {
                'id_entrega': e.id_entrega,
                'hora': e.hora.isoformat(),
                'destino': {
                    'longitud': float(e.destino_longitud),
                    'latitud': float(e.destino_latitud)
                }
            }
            # Solo añadir origen si existe
            if e.dni_origen:
                item['origen'] = {
                    'longitud': float(e.origen_longitud) if e.origen_longitud else None,
                    'latitud': float(e.origen_latitud) if e.origen_latitud else None
                }
            ubicaciones_data.append(item)

        return jsonify({
            'success': True,
            'data': ubicaciones_data,
            'count': len(ubicaciones_data)
        }), 200

    except SQLAlchemyError as e:
        db.session.rollback()
        return jsonify({
            'success': False,
            'error': 'Database error',
            'message': str(e)
        }), 500


@entrega_api.route('/entregas/completar/<int:id_entrega>', methods=['PUT'])
def completar_entrega(id_entrega):
    try:
        entrega = Entrega.query.get(id_entrega)
        if not entrega:
            return jsonify({
                'success': False,
                'error': 'Entrega no encontrada'
            }), 404

        entrega.completado = True
        db.session.commit()

        return jsonify({
            'success': True,
            'message': f'Entrega {id_entrega} marcada como completada.'
        }), 200

    except SQLAlchemyError as e:
        db.session.rollback()
        return jsonify({
            'success': False,
            'error': 'Database error',
            'message': str(e)
        }), 500