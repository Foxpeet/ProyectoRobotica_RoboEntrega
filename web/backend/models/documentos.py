from extensions import db

class Documento(db.Model):
    __tablename__ = 'documentos'

    id_documentos = db.Column(db.Integer, primary_key=True)
    mensaje = db.Column(db.String(45), nullable=True)
    entregas_id_entrega = db.Column(db.Integer, db.ForeignKey('entregas.id_entrega'), nullable=False)
