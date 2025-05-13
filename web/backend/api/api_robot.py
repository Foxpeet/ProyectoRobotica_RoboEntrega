from flask import Blueprint, request, jsonify
from models.robot import Robot
from extensions import db

# Definir el Blueprint para la API de robots
robot_api = Blueprint('api_robot', __name__)

# Ruta para crear un robot (POST)
@robot_api.route('/robots', methods=['POST'])
def create_robot():
    # Obtener los datos del robot desde el cuerpo de la solicitud
    data = request.get_json()

    # Verificar si los campos necesarios están presentes
    if 'modelo_robot' not in data or 'numero_serie_robot' not in data or 'bateria' not in data:
        return jsonify({'error': 'Faltan campos necesarios'}), 400

    # Crear una instancia del modelo Robot
    new_robot = Robot(
        modelo_robot=data['modelo_robot'],
        numero_serie_robot=data['numero_serie_robot'],
        bateria=data['bateria']
    )

    # Guardar el nuevo robot en la base de datos
    db.session.add(new_robot)
    db.session.commit()

    # Responder con el robot creado
    return jsonify({
        'id_robot': new_robot.id_robot,
        'modelo_robot': new_robot.modelo_robot,
        'numero_serie_robot': new_robot.numero_serie_robot,
        'bateria': new_robot.bateria
    }), 201

# Ruta para obtener todos los robots (GET)
@robot_api.route('/robots', methods=['GET'])
def get_robots():
    robots = Robot.query.all()  # Obtener todos los robots de la base de datos
    robots_list = [{
        'id_robot': robot.id_robot,
        'modelo_robot': robot.modelo_robot,
        'numero_serie_robot': robot.numero_serie_robot,
        'bateria': robot.bateria
    } for robot in robots]

    return jsonify(robots_list), 200
