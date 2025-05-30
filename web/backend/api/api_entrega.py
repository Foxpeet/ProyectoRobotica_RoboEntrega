from flask import Blueprint, request, jsonify
from extensions import db
from models.entregas import Entrega
from models.trabajador import Trabajador
from models.paquetes import Paquete
from models.documentos import Documento
from models.mesa import Mesa
from datetime import datetime
from sqlalchemy.exc import SQLAlchemyError

entrega_api = Blueprint('api_entrega', __name__)

# Ruta para obtener entregas no completadas y sus ubicaciones
@entrega_api.route('/entregas/nocompletadas/ubicaciones', methods=['GET'])
def get_entregas_ubicaciones():
    try:
        # Alias para los joins (cuando usamos la misma tabla varias veces)
        TrabajadorOrigen = db.aliased(Trabajador)
        TrabajadorDestino = db.aliased(Trabajador)
        MesaOrigen = db.aliased(Mesa)
        MesaDestino = db.aliased(Mesa)

        # Query para obtener las entregas no completadas y sus ubicaciones
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
            Entrega.completado == False  # Solo entregas no completadas
        ).order_by(
            Entrega.hora.asc()  # Ordenar por hora (más antiguas primero)
        ).all()

        # Formatear las entregas con sus ubicaciones
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
    except Exception as e:
        return jsonify({
            'success': False,
            'error': 'Unknown error',
            'message': str(e)
        }), 500


# Ruta para marcar una entrega como completada
@entrega_api.route('/entregas/completar/<int:id_entrega>', methods=['PUT'])
def completar_entrega(id_entrega):
    try:
        # Obtener la entrega por ID
        entrega = Entrega.query.get(id_entrega)
        if not entrega:
            return jsonify({
                'success': False,
                'error': 'Entrega no encontrada'
            }), 404

        # Marcar la entrega como completada
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
    except Exception as e:
        return jsonify({
            'success': False,
            'error': 'Unknown error',
            'message': str(e)
        }), 500


# Ruta para obtener las entregas más antiguas
@entrega_api.route('/entregas', methods=['GET'])
def obtener_entregas_ordenadas():
    try:
        # Obtener las entregas ordenadas por hora (más antiguas primero)
        entregas = Entrega.query.order_by(Entrega.hora.asc()).all()

        # Crear una lista con los datos que quieres devolver
        entregas_data = [
            {
                'id_entrega': e.id_entrega,
                'hora': e.hora.strftime('%H:%M:%S'),  # Formato de hora
                'tipo': e.tipo,
                'completado': e.completado,
                'robot_id_robot': e.robot_id_robot,
                'paquetes': [paquete.id_paquete for paquete in e.paquetes],  # Listado de IDs de paquetes asociados
                'documentos': [documento.id_documento for documento in e.documentos]  # Listado de IDs de documentos asociados
            }
            for e in entregas
        ]

        return jsonify({
            'success': True,
            'data': entregas_data,
            'count': len(entregas_data)
        }), 200

    except Exception as e:
        return jsonify({
            'success': False,
            'error': 'Unknown error',
            'message': str(e)
        }), 500




@entrega_api.route('/entregas', methods=['POST'])
def crear_entrega():
    try:
        # Obtener los datos del cuerpo de la solicitud
        data = request.get_json()

        # Verificar que los datos necesarios estén presentes
        id_entrega = data.get('id_entrega')
        if not id_entrega:
            return jsonify({
                'success': False,
                'error': 'El campo id_entrega es obligatorio.'
            }), 400
        
        hora = data.get('hora')
        if not hora:
            return jsonify({
                'success': False,
                'error': 'El campo hora es obligatorio.'
            }), 400
        
        tipo = data.get('tipo')
        if not tipo:
            return jsonify({
                'success': False,
                'error': 'El campo tipo es obligatorio.'
            }), 400
        
        dni_destino = data.get('dni_destino')
        if not dni_destino:
            return jsonify({
                'success': False,
                'error': 'El campo dni_destino es obligatorio.'
            }), 400
        
        # Se puede tener o "paquetes" o "documentos" pero no ambos
        if not data.get('paquetes') and not data.get('documentos'):
            return jsonify({
                'success': False,
                'error': 'Se debe proporcionar al menos un paquete o un documento.'
            }), 400

        # Verificar que el robot_id_robot esté presente
        robot_id_robot = data.get('robot_id_robot')
        if not robot_id_robot:
            return jsonify({
                'success': False,
                'error': 'El campo robot_id_robot es obligatorio.'
            }), 400

        # Crear una nueva entrega con los datos proporcionados
        nueva_entrega = Entrega(
            id_entrega=id_entrega,
            hora=hora,
            tipo=tipo,
            dni_origen=data.get('dni_origen'),
            dni_destino=dni_destino,
            robot_id_robot=robot_id_robot
        )

        # Asignar los paquetes o documentos si se proporcionaron
        if data.get('paquetes'):
            for paquete_id in data['paquetes']:
                paquete = Paquete.query.get(paquete_id)
                if paquete:
                    nueva_entrega.paquetes.append(paquete)

        if data.get('documentos'):
            for documento_id in data['documentos']:
                documento = Documento.query.get(documento_id)
                if documento:
                    nueva_entrega.documentos.append(documento)

        # Guardar la nueva entrega
        db.session.add(nueva_entrega)
        db.session.commit()

        return jsonify({
            'success': True,
            'message': 'Entrega creada exitosamente.'
        }), 201

    except Exception as e:
        db.session.rollback()
        return jsonify({
            'success': False,
            'error': 'Error al crear la entrega',
            'message': str(e)
        }), 500