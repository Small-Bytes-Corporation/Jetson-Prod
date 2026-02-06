"""
UDP server for broadcasting camera data.
"""

import json
import socket
import time
from .config import UDP_PORT, UDP_HOST

try:
    import netifaces
    HAS_NETIFACES = True
except ImportError:
    HAS_NETIFACES = False


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
    UDP server for broadcasting camera data.
    Uses UDP broadcast to send camera data to clients.
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
        self.running = False
        
        # Track last emitted data for dashboard
        self.last_sensor_data_time = None
        self.last_status_time = None
        self.last_sensor_data_size = None

    def _get_broadcast_address(self):
        """Get the broadcast address of the active network interface."""
        if HAS_NETIFACES:
            try:
                # Try to get default gateway interface
                gateways = netifaces.gateways()
                default_interface = gateways.get('default', {}).get(netifaces.AF_INET, {}).get(1)
                
                if default_interface:
                    addrs = netifaces.ifaddresses(default_interface)
                    if netifaces.AF_INET in addrs:
                        for addr_info in addrs[netifaces.AF_INET]:
                            if 'broadcast' in addr_info:
                                return addr_info['broadcast']
                
                # Fallback: try all interfaces
                for interface in netifaces.interfaces():
                    addrs = netifaces.ifaddresses(interface)
                    if netifaces.AF_INET in addrs:
                        for addr_info in addrs[netifaces.AF_INET]:
                            if 'broadcast' in addr_info:
                                return addr_info['broadcast']
            except Exception:
                pass
        
        # Final fallback: use configured host or default broadcast
        return self.host if self.host != '255.255.255.255' else '255.255.255.255'

    def _setup_socket(self):
        """Setup UDP socket for sending camera data."""
        try:
            # Determine broadcast address
            self.broadcast_addr = self._get_broadcast_address()
            
            # Socket for sending (broadcast)
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # Bind to all interfaces for sending
            self.send_socket.bind(('0.0.0.0', 0))
            
            if not self.suppress_debug_prints:
                print(f"[UDP] Using broadcast address: {self.broadcast_addr}")
            
            return True
        except Exception as e:
            if not self.suppress_debug_prints:
                print(f"[UDP] Error setting up socket: {e}")
            # Fallback: try without netifaces
            try:
                self.broadcast_addr = '255.255.255.255'
                self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                self.send_socket.bind(('0.0.0.0', 0))
                if not self.suppress_debug_prints:
                    print(f"[UDP] Fallback: Using {self.broadcast_addr} as broadcast address")
                return True
            except Exception as e2:
                if not self.suppress_debug_prints:
                    print(f"[UDP] Fallback also failed: {e2}")
                return False

    def start(self):
        """Start the UDP server."""
        if self.running:
            if not self.suppress_debug_prints:
                print("[UDP] Server already running")
            return

        if not self._setup_socket():
            if not self.suppress_debug_prints:
                print("[UDP] Failed to setup socket")
            return

        self.running = True

        if not self.suppress_debug_prints:
            print(f"[UDP] Starting server on {self.host}:{self.port} (broadcast)")

    def stop(self):
        """Stop the UDP server."""
        if not self.running:
            return

        self.running = False

        try:
            if self.send_socket:
                self.send_socket.close()
        except Exception as e:
            if not self.suppress_debug_prints:
                print(f"[UDP] Error stopping server: {e}")

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
            # Use determined broadcast address
            broadcast_addr = getattr(self, 'broadcast_addr', self.host)
            self.send_socket.sendto(message_bytes, (broadcast_addr, self.port))
            
            return True
        except Exception as e:
            if not self.suppress_debug_prints:
                print(f"[UDP] Error sending {message_type}: {e}")
            return False

    def emit_sensor_data(self, data):
        """Emit sensor data via UDP broadcast."""
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
        """Emit status information via UDP broadcast."""
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
