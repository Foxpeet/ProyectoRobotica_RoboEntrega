from flask import Blueprint, request, jsonify
from sqlalchemy.exc import IntegrityError
from extensions import db
from models.mesa import Mesa

mesa_api = Blueprint('api_mesa', __name__)

@mesa_api.route('/mesas', methods=['GET'])
def get_mesas():
    mesas = Mesa.query.all()
    return jsonify([
        {
            'id_mesa': m.id_mesa,
            'longitud_x': str(m.longitud_x),
            'latitud_y': str(m.latitud_y)
        }
        for m in mesas
    ])

@mesa_api.route('/mesas', methods=['POST'])
def create_mesa():
    data = request.get_json()

    nueva_mesa = Mesa(
        longitud_x=data['longitud_x'],
        latitud_y=data['latitud_y']
    )
    
    db.session.add(nueva_mesa)

    try:
        db.session.commit()
    except IntegrityError as e:
        db.session.rollback()

        # Versión robusta para SQLite
        if 'UNIQUE constraint failed: mesa.longitud_x, mesa.latitud_y' in str(e.orig):
            return jsonify({'message': 'Este lugar ya tiene una cabina.'}), 400
        else:
            return jsonify({'message': 'Error al guardar la mesa.'}), 500

    return jsonify({'message': 'Mesa añadida correctamente'}), 201
