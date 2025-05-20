from extensions import db

class Trabajador(db.Model):
    __tablename__ = 'trabajador'

    dni_trabajador = db.Column(db.String(9), primary_key=True)
    nombre_trabajador = db.Column(db.String(45), nullable=False)
    apellido_trabajador = db.Column(db.String(45), nullable=False)
    correo = db.Column(db.String(45), unique=True, nullable=False)
    contraseña_hash = db.Column(db.String(100), unique=True, nullable=False)
    presente = db.Column(db.Boolean, default=False, nullable=False)
    rol_admin = db.Column(db.Boolean, default=False, nullable=False)
    mesa_id_mesa = db.Column(db.Integer, db.ForeignKey('mesa.id_mesa'), nullable=False)
