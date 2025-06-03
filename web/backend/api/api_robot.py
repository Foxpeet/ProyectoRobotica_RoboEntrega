from flask import Blueprint, request, jsonify
from models.robot import Robot
from extensions import db

robot_api = Blueprint('api_robot', __name__)

@robot_api.route('/robots', methods=['POST'])
def create_robot():
    data = request.get_json()

    if 'modelo_robot' not in data or 'numero_serie_robot' not in data:
        return jsonify({'error': 'Faltan campos necesarios'}), 400

    new_robot = Robot(
        modelo_robot=data['modelo_robot'],
        numero_serie_robot=data['numero_serie_robot'],
    )

    db.session.add(new_robot)
    db.session.commit()

    return jsonify({
        'id_robot': new_robot.id_robot,
        'modelo_robot': new_robot.modelo_robot,
        'numero_serie_robot': new_robot.numero_serie_robot,
    }), 201

@robot_api.route('/robots', methods=['GET'])
def get_robots():
    robots = Robot.query.all()
    robots_list = [{
        'id_robot': robot.id_robot,
        'modelo_robot': robot.modelo_robot,
        'numero_serie_robot': robot.numero_serie_robot,
    } for robot in robots]

    return jsonify(robots_list), 200

@robot_api.route('/robots/<int:id_robot>', methods=['DELETE'])
def eliminar_robot(id_robot):
    robot = Robot.query.get(id_robot)
    
    if not robot:
        return jsonify({'message': 'Robot no encontrado'}), 404
    
    try:
        db.session.delete(robot)
        db.session.commit()
        return jsonify({'message': f'Robot ID {id_robot} eliminado con éxito'}), 200
    except Exception as e:
        db.session.rollback()
        return jsonify({'message': f'Error al eliminar el robot: {str(e)}'}), 500