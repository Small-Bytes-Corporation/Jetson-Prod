"""
Socket.io server for broadcasting sensor data.
"""

import threading
from flask import Flask
from flask_socketio import SocketIO, emit
from .config import SOCKETIO_PORT, SOCKETIO_HOST, SOCKETIO_CORS_ORIGINS


class SocketServer:
    """
    Socket.io server for broadcasting lidar and camera data to clients.
    """
    
    def __init__(self, port=SOCKETIO_PORT, host=SOCKETIO_HOST, cors_origins=SOCKETIO_CORS_ORIGINS):
        """
        Initialize the socket server.
        
        Args:
            port: Port to listen on.
            host: Host to bind to.
            cors_origins: CORS origins allowed.
        """
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
        
        # Setup event handlers
        self._setup_handlers()
    
    def _setup_handlers(self):
        """Setup socket.io event handlers."""
        
        @self.socketio.on('connect')
        def handle_connect():
            """Handle client connection."""
            print(f"[SocketIO] Client connected: {self.socketio.server.manager.get_namespace('/').get_participants('/')}")
            emit('status', {
                'message': 'Connected to robocar sensor stream',
                'connected': True
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """Handle client disconnection."""
            print("[SocketIO] Client disconnected")
        
        @self.socketio.on('ping')
        def handle_ping():
            """Handle ping from client."""
            emit('pong', {'timestamp': self.socketio.server.get_time()})
    
    def start(self):
        """Start the socket server in a separate thread."""
        if self.running:
            print("[SocketIO] Server already running")
            return
        
        self.running = True
        
        def run_server():
            """Run the Flask-SocketIO server."""
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
        
        # Wait a bit for server to start
        import time
        time.sleep(0.5)
    
    def stop(self):
        """Stop the socket server."""
        if not self.running:
            return
        
        self.running = False
        try:
            # Stop the socketio server
            self.socketio.stop()
        except Exception as e:
            print(f"[SocketIO] Error stopping server: {e}")
        
        print("[SocketIO] Server stopped")
    
    def emit_sensor_data(self, data):
        """
        Emit sensor data to all connected clients.
        
        Args:
            data: Dictionary containing sensor data (lidar + camera).
        """
        if self.running:
            try:
                self.socketio.emit('sensor_data', data)
            except Exception as e:
                print(f"[SocketIO] Error emitting sensor_data: {e}")
    
    def emit_status(self, status):
        """
        Emit status information to all connected clients.
        
        Args:
            status: Dictionary containing status information.
        """
        if self.running:
            try:
                self.socketio.emit('status', status)
            except Exception as e:
                print(f"[SocketIO] Error emitting status: {e}")
    
    def get_client_count(self):
        """
        Get the number of connected clients.
        
        Returns:
            int: Number of connected clients.
        """
        try:
            namespace = self.socketio.server.manager.get_namespace('/')
            return len(namespace.get_participants('/'))
        except Exception:
            return 0
    
    def is_running(self):
        """
        Check if server is running.
        
        Returns:
            bool: True if server is running, False otherwise.
        """
        return self.running
