from extensions import db

class Robot(db.Model):
    __tablename__ = 'robot'

    id_robot = db.Column(db.Integer, primary_key=True)
    modelo_robot = db.Column(db.String(50), nullable=False)
    numero_serie_robot = db.Column(db.String(100), unique=True, nullable=False)
    bateria = db.Column(db.Boolean, nullable=False)

    # Relación con el modelo Entrega
    entregas = db.relationship("Entrega", backref="robot")

    def __repr__(self):
        return f'<Robot {self.modelo_robot}>'
