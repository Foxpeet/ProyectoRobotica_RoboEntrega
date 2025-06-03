from extensions import db

class Entrega(db.Model):
    __tablename__ = 'entregas'

    id_entrega = db.Column(db.String(45), primary_key=True)
    hora = db.Column(db.Time, nullable=False)  # Cambio aquí: 'hora' es ahora un campo Time
    tipo = db.Column(db.String(45), nullable=False)
    dni_origen = db.Column(db.String(45), nullable=True)
    dni_destino = db.Column(db.String(45), nullable=False)
    completado = db.Column(db.Boolean, default=False, nullable=False)

    robot_id_robot = db.Column(db.Integer, db.ForeignKey('robot.id_robot'), nullable=False)
