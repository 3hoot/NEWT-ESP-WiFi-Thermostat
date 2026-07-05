import threading
import socket
import json
from flask import Flask, jsonify, render_template, request

# --- Configuration ---
HOST = '0.0.0.0'
WEB_PORT = 8000
SOCKET_PORT = 8080

# --- Initialize Flask App ---
app = Flask(__name__)

# needed output data
thermostat_state = {
    "temperature": None,
    "setpoint": 22.0,
    "running": False,
    "output": False,
    "mode": "Stopped",
    "esp_connected": False
}

# Protects data read by Flask and modified by the socket thread.
state_lock = threading.Lock()

# Protects the active ESP32 TCP connection.
connection_lock = threading.Lock()

# Stores the currently connected ESP32 socket.
esp_connection = None

# --- Web server routes ---
@app.route('/')
def index():
    # Simple HTML template served directly
    return render_template('web.html')

@app.route("/api/status")
def api_status():
    with state_lock:
        return jsonify(dict(thermostat_state))
    
@app.route("/api/control", methods=["POST"])
def api_control():
    data = request.get_json(silent=True) or {}

    action = data.get("action")
    value = data.get("value")

    if action == "setpoint":
        try:
            value = float(value)
        except (TypeError, ValueError):
            return jsonify({
                "success": False,
                "message": "Invalid temperature value."
            }), 400

        if value < 5 or value > 35:
            return jsonify({
                "success": False,
                "message": "Temperature must be between 5 and 35 °C."
            }), 400

        message_for_esp = {
            "command": "setpoint",
            "value": value
        }

    elif action == "start":
        message_for_esp = {
            "command": "start"
        }

    elif action == "stop":
        message_for_esp = {
            "command": "stop"
        }

    else:
        return jsonify({
            "success": False,
            "message": "Unknown command."
        }), 400

    sent = send_to_esp(message_for_esp)

    if not sent:
        return jsonify({
            "success": False,
            "message": "ESP32 is not connected."
        }), 503

    return jsonify({
        "success": True,
        "message": "Command sent to ESP32."
    })

def run_web_server():
    app.run(host=HOST, port=WEB_PORT, debug=False, use_reloader=False)


# collecting data from the esp
def handle_esp_client(conn, addr):
    global esp_connection

    print(f"ESP32 connected: {addr}")

    with connection_lock:
        esp_connection = conn

    with state_lock:
        thermostat_state["esp_connected"] = True

    buffer = ""

    try:
        while True:
            received = conn.recv(1024)

            if not received:
                break

            buffer += received.decode("utf-8")

            # One JSON message ends with \n
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                try:
                    message = json.loads(line)

                    print(f"Received from ESP32: {message}")

                    with state_lock:
                        if "temperature" in message:
                            thermostat_state["temperature"] = message["temperature"]

                        if "setpoint" in message:
                            thermostat_state["setpoint"] = message["setpoint"]

                        if "running" in message:
                            thermostat_state["running"] = message["running"]

                        if "output" in message:
                            thermostat_state["output"] = message["output"]

                        if "mode" in message:
                            thermostat_state["mode"] = message["mode"]

                except json.JSONDecodeError:
                    print(f"Invalid JSON received: {line}")

    except OSError as error:
        print(f"Socket error: {error}")

    finally:
        print(f"ESP32 disconnected: {addr}")

        with connection_lock:
            if esp_connection == conn:
                esp_connection = None

        with state_lock:
            thermostat_state["esp_connected"] = False
            thermostat_state["running"] = False
            thermostat_state["output"] = False

        conn.close()



#sending data to the esp
def send_to_esp(message):
    global esp_connection

    text = json.dumps(message) + "\n"

    with connection_lock:
        if esp_connection is None:
            return False

        try:
            esp_connection.sendall(text.encode("utf-8"))

            print(f"Sent to ESP32: {text.strip()}")
            return True

        except OSError:
            print("Connection to ESP32 lost.")

            esp_connection = None

    with state_lock:
        thermostat_state["esp_connected"] = False
        thermostat_state["running"] = False
        thermostat_state["output"] = False

    return False


# --- Socket server ---
def run_socket_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        server_socket.bind((HOST, SOCKET_PORT))
        server_socket.listen()

        print(f"Socket server listening on {SOCKET_PORT}...")

        while True:
            conn, addr = server_socket.accept()

            client_thread = threading.Thread(
                target=handle_esp_client,
                args=(conn, addr),
                daemon=True
            )

            client_thread.start()


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
