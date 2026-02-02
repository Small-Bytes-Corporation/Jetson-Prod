"""
Socket.io server for broadcasting sensor data.
"""

import threading
from flask import Flask, request
from flask_socketio import SocketIO, emit
from .config import SOCKETIO_PORT, SOCKETIO_HOST, SOCKETIO_CORS_ORIGINS


class SocketServer:
    """
    Socket.io server for broadcasting lidar and camera data to clients.
    """

    def __init__(self, port=SOCKETIO_PORT, host=SOCKETIO_HOST, cors_origins=SOCKETIO_CORS_ORIGINS):
        self.port = port
        self.host = host
        self.cors_origins = cors_origins

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

            emit('status', {
                'message': 'Connected to robocar sensor stream',
                'connected': True
            })

        @self.socketio.on('disconnect')
        def handle_disconnect():
            sid = request.sid
            self.clients.discard(sid)

            print(f"[SocketIO] Client disconnected: {sid} ({len(self.clients)} clients)")

        @self.socketio.on('ping')
        def handle_ping():
            emit('pong', {'timestamp': self.socketio.server.eio.get_time()})

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
                    use_reloader=False,
                    allow_unsafe_werkzeug=True
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
            self.socketio.stop()
        except Exception as e:
            print(f"[SocketIO] Error stopping server: {e}")

        print("[SocketIO] Server stopped")

    def emit_sensor_data(self, data):
        """Emit sensor data to all connected clients."""
        if self.running:
            try:
                self.socketio.emit('sensor_data', data)
            except Exception as e:
                print(f"[SocketIO] Error emitting sensor_data: {e}")

    def emit_status(self, status):
        """Emit status information to all connected clients."""
        if self.running:
            try:
                self.socketio.emit('status', status)
            except Exception as e:
                print(f"[SocketIO] Error emitting status: {e}")

    def get_client_count(self):
        """Return number of connected clients."""
        return len(self.clients)

    def is_running(self):
        return self.running
