"""
Socket.io server for broadcasting sensor data.
"""

import json
import threading
from flask import Flask, request
from flask_socketio import SocketIO, emit
from .config import SOCKETIO_PORT, SOCKETIO_HOST, SOCKETIO_CORS_ORIGINS


def _payload_for_debug(data, max_str_len=200, max_list_len=10):
    """
    Build a copy of the payload with large values summarized for console debug.
    Converts non-JSON-serializable values to repr so json.dumps never fails.
    """
    if not isinstance(data, dict):
        try:
            json.dumps(data)
            return data
        except (TypeError, ValueError):
            return repr(data)
    out = {}
    for k, v in data.items():
        if isinstance(v, dict):
            out[k] = _payload_for_debug(v, max_str_len, max_list_len)
        elif isinstance(v, str) and len(v) > max_str_len:
            out[k] = f"<str len={len(v)}>"
        elif isinstance(v, list) and len(v) > max_list_len:
            out[k] = f"<list len={len(v)}>"
        else:
            try:
                json.dumps(v)
                out[k] = v
            except (TypeError, ValueError):
                out[k] = repr(v)
    return out


class SocketServer:
    """
    Socket.io server for broadcasting camera data to clients.
    """

    def __init__(self, port=SOCKETIO_PORT, host=SOCKETIO_HOST, cors_origins=SOCKETIO_CORS_ORIGINS, debug_payload=False):
        self.port = port
        self.host = host
        self.cors_origins = cors_origins
        self.debug_payload = debug_payload

        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'robocar-secret-key'

        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins=cors_origins,
            async_mode='threading',
            logger=False,
            engineio_logger=False
        )

        self.server_thread = None
        self.running = False

        # Track connected clients
        self.clients = set()

        self._setup_handlers()

    def _setup_handlers(self):
        """Setup socket.io event handlers."""

        @self.socketio.on('connect')
        def handle_connect(auth=None):
            sid = request.sid
            self.clients.add(sid)

            print(f"[SocketIO] Client connected: {sid} ({len(self.clients)} clients)")

            status_payload = {
                'message': 'Connected to robocar sensor stream',
                'connected': True
            }
            if self.debug_payload:
                print(f"[SocketIO] [debug] emit status: {json.dumps(status_payload, indent=2)}")
            emit('status', status_payload)

        @self.socketio.on('disconnect')
        def handle_disconnect():
            sid = request.sid
            self.clients.discard(sid)

            print(f"[SocketIO] Client disconnected: {sid} ({len(self.clients)} clients)")

        @self.socketio.on('ping')
        def handle_ping():
            pong_payload = {'timestamp': self.socketio.server.eio.get_time()}
            if self.debug_payload:
                print(f"[SocketIO] [debug] emit pong: {json.dumps(pong_payload, indent=2)}")
            emit('pong', pong_payload)

    def start(self):
        """Start the socket server in a separate thread."""
        if self.running:
            print("[SocketIO] Server already running")
            return

        self.running = True

        def run_server():
            try:
                print(f"[SocketIO] Starting server on {self.host}:{self.port}")
                self.socketio.run(
                    self.app,
                    host=self.host,
                    port=self.port,
                    debug=False,
                    use_reloader=False
                )
            except Exception as e:
                print(f"[SocketIO] Server error: {e}")
                self.running = False

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

        import time
        time.sleep(0.5)

    def stop(self):
        """Stop the socket server."""
        if not self.running:
            return

        self.running = False
        try:
            with self.app.test_request_context():
                self.socketio.stop()
        except Exception as e:
            msg = str(e).lower()
            if "request context" not in msg and "application context" not in msg and "unknown web server" not in msg:
                print(f"[SocketIO] Error stopping server: {e}")
        print("[SocketIO] Server stopped")

    def emit_sensor_data(self, data):
        """Emit sensor data to all connected clients."""
        if self.running:
            try:
                if self.debug_payload:
                    summarized = _payload_for_debug(data)
                    print(f"[SocketIO] [debug] emit sensor_data (résumé console; les clients reçoivent les données complètes):\n{json.dumps(summarized, indent=2)}")
                self.socketio.emit('sensor_data', data)
            except Exception as e:
                print(f"[SocketIO] Error emitting sensor_data: {e}")

    def emit_status(self, status):
        """Emit status information to all connected clients."""
        if self.running:
            try:
                if self.debug_payload:
                    print(f"[SocketIO] [debug] emit status:\n{json.dumps(status, indent=2)}")
                self.socketio.emit('status', status)
            except Exception as e:
                print(f"[SocketIO] Error emitting status: {e}")

    def get_client_count(self):
        """Return number of connected clients."""
        return len(self.clients)

    def is_running(self):
        return self.running
