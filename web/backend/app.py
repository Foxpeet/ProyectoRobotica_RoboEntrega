from flask import Flask
from extensions import db
from flask_migrate import Migrate
from flask_cors import CORS
import os
from api.api_robot import robot_api
from api.api_trabajador import trabajador_api
from api.api_mesa import mesa_api
from api.api_iniciar_sesion import iniciarSesion_api

app = Flask(__name__)

# Ruta absoluta dinámica al archivo SQLite
basedir = os.path.abspath(os.path.dirname(__file__))
db_path = os.path.join(basedir, 'instance', 'robo_entrega.db')
app.config['SQLALCHEMY_DATABASE_URI'] = f'sqlite:///{db_path}'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

# Inicializa la db con la app
db.init_app(app)
migrate = Migrate(app, db)

# Registrar el Blueprint
app.register_blueprint(robot_api, url_prefix='/api')
app.register_blueprint(trabajador_api, url_prefix='/api')
app.register_blueprint(mesa_api, url_prefix='/api')
app.register_blueprint(iniciarSesion_api, url_prefix='/api')

CORS(app)

# Importa los modelos después de inicializar db
with app.app_context():
    from models.robot import Robot
    from models.mesa import Mesa
    from models.trabajador import Trabajador
    from models.entregas import Entrega  
    from models.paquetes import Paquete  
    from models.documentos import Documento
    db.create_all()

if __name__ == '__main__':
    app.run(debug=True)
