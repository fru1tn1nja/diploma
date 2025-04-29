from flask import Flask, render_template
import os

def create_app():
    app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))
    @app.route('/')
    def index():
        return render_template('index.html')
    return app

def main():
    app = create_app()
    app.run(host='0.0.0.0', port=5000, debug=False)
