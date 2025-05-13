from extensions import db

class Paquete(db.Model):
    __tablename__ = 'paquetes'

    id_paquetes = db.Column(db.Integer, primary_key=True)
    dni_destinatario = db.Column(db.String(45), nullable=False)
    entregas_id_entrega = db.Column(db.Integer, db.ForeignKey('entregas.id_entrega'), nullable=False)
