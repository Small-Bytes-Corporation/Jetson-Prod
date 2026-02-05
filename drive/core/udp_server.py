"""
UDP server for broadcasting sensor data.
"""

import json
import socket
import threading
import time
from .config import UDP_PORT, UDP_HOST, UDP_HEARTBEAT_TIMEOUT


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


class UDPServer:
    """
    UDP server for broadcasting sensor data to clients.
    Uses UDP broadcast to send data and heartbeat mechanism to track connected clients.
    """

    def __init__(self, port=UDP_PORT, host=UDP_HOST, debug_payload=False, suppress_debug_prints=False):
        """
        Initialize the UDP server.
        
        Args:
            port: UDP port to use (default: UDP_PORT from config).
            host: Host address for broadcast (default: UDP_HOST from config).
            debug_payload: If True, print payload summaries for debugging.
            suppress_debug_prints: If True, suppress debug print statements.
        """
        self.port = port
        self.host = host
        self.debug_payload = debug_payload
        self.suppress_debug_prints = suppress_debug_prints
        
        # UDP socket for sending data
        self.send_socket = None
        
        # UDP socket for receiving heartbeats
        self.receive_socket = None
        
        # Thread for receiving heartbeats
        self.receive_thread = None
        self.running = False
        
        # Track connected clients: {(ip, port): last_heartbeat_time}
        self.clients = {}
        self.clients_lock = threading.Lock()
        
        # Track last emitted data for dashboard
        self.last_sensor_data_time = None
        self.last_status_time = None
        self.last_sensor_data_size = None
        
        # Cleanup thread for removing stale clients
        self.cleanup_thread = None

    def _setup_sockets(self):
        """Setup UDP sockets for sending and receiving."""
        try:
            # Socket for sending (broadcast)
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            # Socket for receiving heartbeats
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.receive_socket.bind(('', self.port))
            self.receive_socket.settimeout(1.0)  # 1 second timeout for checking running flag
            
            return True
        except Exception as e:
            if not self.suppress_debug_prints:
                print(f"[UDP] Error setting up sockets: {e}")
            return False

    def _receive_heartbeats(self):
        """Thread function to receive heartbeat messages from clients."""
        while self.running:
            try:
                data, addr = self.receive_socket.recvfrom(1024)
                try:
                    message = json.loads(data.decode('utf-8'))
                    if message.get('type') == 'heartbeat':
                        with self.clients_lock:
                            self.clients[addr] = time.time()
                            if not self.suppress_debug_prints:
                                print(f"[UDP] Heartbeat received from {addr} ({len(self.clients)} clients)")
                except (json.JSONDecodeError, UnicodeDecodeError, KeyError):
                    # Ignore invalid messages
                    pass
            except socket.timeout:
                # Timeout is expected, continue loop to check running flag
                continue
            except Exception as e:
                if self.running:  # Only print if we're supposed to be running
                    if not self.suppress_debug_prints:
                        print(f"[UDP] Error receiving heartbeat: {e}")

    def _cleanup_stale_clients(self):
        """Thread function to remove clients that haven't sent heartbeats recently."""
        while self.running:
            try:
                current_time = time.time()
                with self.clients_lock:
                    stale_clients = [
                        addr for addr, last_time in self.clients.items()
                        if current_time - last_time > UDP_HEARTBEAT_TIMEOUT
                    ]
                    for addr in stale_clients:
                        del self.clients[addr]
                        if not self.suppress_debug_prints:
                            print(f"[UDP] Client {addr} timed out ({len(self.clients)} clients)")
                time.sleep(1.0)  # Check every second
            except Exception as e:
                if self.running:
                    if not self.suppress_debug_prints:
                        print(f"[UDP] Error in cleanup thread: {e}")

    def start(self):
        """Start the UDP server."""
        if self.running:
            if not self.suppress_debug_prints:
                print("[UDP] Server already running")
            return

        if not self._setup_sockets():
            if not self.suppress_debug_prints:
                print("[UDP] Failed to setup sockets")
            return

        self.running = True

        # Start thread for receiving heartbeats
        self.receive_thread = threading.Thread(target=self._receive_heartbeats, daemon=True)
        self.receive_thread.start()

        # Start thread for cleaning up stale clients
        self.cleanup_thread = threading.Thread(target=self._cleanup_stale_clients, daemon=True)
        self.cleanup_thread.start()

        if not self.suppress_debug_prints:
            print(f"[UDP] Starting server on {self.host}:{self.port} (broadcast)")

    def stop(self):
        """Stop the UDP server."""
        if not self.running:
            return

        self.running = False

        try:
            if self.receive_socket:
                self.receive_socket.close()
            if self.send_socket:
                self.send_socket.close()
        except Exception as e:
            if not self.suppress_debug_prints:
                print(f"[UDP] Error stopping server: {e}")

        # Wait for threads to finish
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        if self.cleanup_thread:
            self.cleanup_thread.join(timeout=2.0)

        with self.clients_lock:
            self.clients.clear()

        if not self.suppress_debug_prints:
            print("[UDP] Server stopped")

    def _send_message(self, message_type, data):
        """Send a UDP message via broadcast."""
        if not self.running or not self.send_socket:
            return

        try:
            # Add message type to data
            payload = {
                "type": message_type,
                **data
            }

            # Encode to JSON and then to bytes
            message_json = json.dumps(payload)
            message_bytes = message_json.encode('utf-8')

            # Send via broadcast
            # Use broadcast address from config (default: 255.255.255.255)
            self.send_socket.sendto(message_bytes, (self.host, self.port))
            
            return True
        except Exception as e:
            if not self.suppress_debug_prints:
                print(f"[UDP] Error sending {message_type}: {e}")
            return False

    def emit_sensor_data(self, data):
        """Emit sensor data to all connected clients via UDP broadcast."""
        if self.running:
            try:
                if self.debug_payload and not self.suppress_debug_prints:
                    summarized = _payload_for_debug(data)
                    print(f"[UDP] [debug] emit sensor_data (résumé console; les clients reçoivent les données complètes):\n{json.dumps(summarized, indent=2)}")
                
                self._send_message("sensor_data", data)
                
                # Track for dashboard
                self.last_sensor_data_time = time.time()
                # Estimate data size
                try:
                    self.last_sensor_data_size = len(json.dumps(data))
                except:
                    self.last_sensor_data_size = None
            except Exception as e:
                if not self.suppress_debug_prints:
                    print(f"[UDP] Error emitting sensor_data: {e}")

    def emit_status(self, status):
        """Emit status information to all connected clients via UDP broadcast."""
        if self.running:
            try:
                if self.debug_payload and not self.suppress_debug_prints:
                    print(f"[UDP] [debug] emit status:\n{json.dumps(status, indent=2)}")
                
                self._send_message("status", status)
                
                # Track for dashboard
                self.last_status_time = time.time()
            except Exception as e:
                if not self.suppress_debug_prints:
                    print(f"[UDP] Error emitting status: {e}")

    def get_client_count(self):
        """Return number of connected clients."""
        with self.clients_lock:
            return len(self.clients)

    def is_running(self):
        """Return whether the server is running."""
        return self.running
    
    def get_last_emission_info(self):
        """Get information about last emissions for dashboard."""
        return {
            'last_sensor_data_time': self.last_sensor_data_time,
            'last_status_time': self.last_status_time,
            'last_sensor_data_size': self.last_sensor_data_size,
        }
