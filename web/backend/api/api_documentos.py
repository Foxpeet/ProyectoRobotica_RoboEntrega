from flask import Blueprint, request, jsonify
from extensions import db
from models import Documento, Entrega

documento_api = Blueprint('documento_api', __name__)

@documento_api.route('/api/documentos', methods=['POST'])
def create_documento():
    data = request.get_json()

    if 'mensaje' not in data or 'id_entrega' not in data:
        return jsonify({'error': 'Faltan campos obligatorios'}), 400

    try:
        entrega = Entrega.query.get(data['id_entrega'])
        if not entrega:
            return jsonify({'error': 'La entrega asociada no existe'}), 404

        nuevo_documento = Documento(
            mensaje=data.get('mensaje', ''),
            entregas_id_entrega=data['id_entrega']
        )
        db.session.add(nuevo_documento)
        db.session.commit()

        return jsonify({
            'message': 'Documento creado exitosamente',
            'id_documento': nuevo_documento.id_documentos
        }), 201

    except Exception as e:
        db.session.rollback()
        return jsonify({'error': str(e)}), 500
