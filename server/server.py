import threading
import socket
import json
from flask import Flask, jsonify, render_template, send_from_directory

# --- Configuration ---
HOST = '0.0.0.0'
WEB_PORT = 8000
SOCKET_PORT = 8080

# --- Initialize Flask App ---
app = Flask(__name__)


# --- Web server routes ---
@app.route('/')
def index():
    # Simple HTML template served directly
    return render_template('web.html')


def run_web_server():
    app.run(host=HOST, port=WEB_PORT, debug=False, use_reloader=False)


# --- Socket server ---
def run_socket_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, SOCKET_PORT))
        s.listen()
        print(f"Socket server listening on {SOCKET_PORT}...")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"ESP32 connected: {addr}")


# --- Main Threading ---
if __name__ == "__main__":
    t1 = threading.Thread(target=run_web_server, daemon=True)
    t2 = threading.Thread(target=run_socket_server, daemon=True)

    t1.start()
    t2.start()

    print(
        f"Server started. Web: http://localhost:{WEB_PORT}, Socket: {SOCKET_PORT}")

    try:
        while True:
            threading.Event().wait(1)
    except KeyboardInterrupt:
        print("Shutting down...")
