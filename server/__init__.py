from flask import Flask
from flask_socketio import SocketIO

socketio = SocketIO()


def create_app(debug=False):
    """Create an application."""
    app = Flask(__name__)
    app.debug = debug
    app.config["SECRET_KEY"] = "gjr39dkjn344_!67#"

    from .routes import api_bp as api_blueprint

    app.register_blueprint(api_blueprint)

    socketio.init_app(app)
    return app
