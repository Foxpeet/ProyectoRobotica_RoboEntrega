from extensions import db

class Mesa(db.Model):
    __tablename__ = 'mesa'

    id_mesa = db.Column(db.Integer, primary_key=True)
    longitud_x = db.Column(db.Numeric(9, 6), nullable=False)
    latitud_y = db.Column(db.Numeric(9, 6), nullable=False)

    trabajadores = db.relationship("Trabajador", backref="mesa")

    __table_args__ = (db.UniqueConstraint('longitud_x', 'latitud_y', name='uniq_lat_lon'),)
