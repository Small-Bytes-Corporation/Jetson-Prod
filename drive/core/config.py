"""
Configuration constants for the robocar control system.
"""

import platform
import glob
import os

# Motor control constants
MAX_SPEED = 0.15
DELTA_ACC = 0.003
DELTA_BRAKE = 0.006
# DEFAULT_SERIAL_PORT will be set after loading ports_config.py
DEFAULT_SERIAL_PORT = None  # Will be set below

# Autonomous navigation constants
AUTONOMOUS_MAX_SPEED = 0.05  # Maximum speed for autonomous mode (same as MAX_SPEED by default)
AUTONOMOUS_MIN_SPEED = 0.03  # Minimum speed for autonomous mode
AUTONOMOUS_SAFETY_DISTANCE = 0.4  # Safety bubble radius (meters)
AUTONOMOUS_FIELD_OF_VIEW = (-90.0, 90.0)  # Field of view in degrees (left, right)
AUTONOMOUS_LOOKAHEAD_DISTANCE = 300.0  # Preferred forward distance (meters)
AUTONOMOUS_GAP_THRESHOLD = 1.0  # Minimum gap size to consider (meters)

# Camera constants
CAM_WIDTH = 320
CAM_HEIGHT = 180
CAM_FPS = 15
CAMERA_STARTUP_DELAY = 0.5  # Delay in seconds after camera initialization before starting DataPublisher (helps on Jetson)

# Platform detection
IS_MAC = platform.system() == 'Darwin'
IS_LINUX = platform.system() == 'Linux'


def _get_available_serial_ports():
    """
    Get list of available serial ports based on the platform.
    
    On Mac, prefers /dev/tty.* ports over /dev/cu.* ports because
    /dev/cu.* ports don't support all termios configurations.
    
    Returns:
        list: List of available serial port paths (sorted, tty ports first on Mac).
    """
    ports = []
    if IS_MAC:
        # On Mac, prefer /dev/tty.* ports (they support termios configuration)
        # /dev/cu.* ports are call-out ports that don't support all configurations
        tty_patterns = [
            '/dev/tty.usbserial-*',
            '/dev/tty.usbmodem*'
        ]
        cu_patterns = [
            '/dev/cu.usbserial-*',
            '/dev/cu.usbmodem*'
        ]
        # First add tty ports (preferred)
        for pattern in tty_patterns:
            ports.extend(glob.glob(pattern))
        # Then add cu ports (fallback)
        for pattern in cu_patterns:
            ports.extend(glob.glob(pattern))
    else:
        # On Linux/Jetson, check common USB serial ports
        for port_pattern in ['/dev/ttyUSB*', '/dev/ttyACM*']:
            ports.extend(glob.glob(port_pattern))
    
    # Filter out non-existent ports
    ports = [p for p in ports if os.path.exists(p)]
    
    # On Mac, sort to ensure tty ports come before cu ports
    if IS_MAC:
        tty_ports = [p for p in ports if '/dev/tty.' in p]
        cu_ports = [p for p in ports if '/dev/cu.' in p]
        return sorted(tty_ports) + sorted(cu_ports)
    else:
        return sorted(ports)


# Load port configuration from ports_config.py if available
try:
    import sys
    import importlib.util
    # Get project root (parent of drive/core/)
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    ports_config_path = os.path.join(project_root, 'ports_config.py')
    if os.path.exists(ports_config_path):
        spec = importlib.util.spec_from_file_location("ports_config", ports_config_path)
        ports_config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ports_config)
        # Use ports from ports_config.py
        _VESC_PORT = getattr(ports_config, 'VESC_PORT', '/dev/ttyACM1')
        _LIDAR_PORT = getattr(ports_config, 'LIDAR_PORT', '/dev/ttyUSB0')
        _PAN_TILT_PORT = getattr(ports_config, 'PAN_TILT_PORT', '/dev/ttyACM2')
        _CAMERA_DEVICE_ID = getattr(ports_config, 'CAMERA_DEVICE_ID', None)
    else:
        # Fallback to defaults if ports_config.py doesn't exist
        _VESC_PORT = '/dev/ttyACM1'
        _LIDAR_PORT = '/dev/ttyUSB0'
        _PAN_TILT_PORT = '/dev/ttyACM2'
        _CAMERA_DEVICE_ID = None
except Exception:
    # Fallback to defaults if import fails
    _VESC_PORT = '/dev/ttyACM1'
    _LIDAR_PORT = '/dev/ttyUSB0'
    _PAN_TILT_PORT = '/dev/ttyACM2'
    _CAMERA_DEVICE_ID = None

# Lidar constants (default from ports_config.py)
DEFAULT_LIDAR_PORT = _LIDAR_PORT

LIDAR_BAUDRATE = 230400  # À vérifier selon documentation D500

# UDP network constants
UDP_PORT = 3000
UDP_HOST = '255.255.255.255'  # Broadcast address
UDP_HEARTBEAT_TIMEOUT = 3.0  # Seconds before considering a client disconnected

# Legacy SocketIO constants (deprecated, kept for compatibility during transition)
SOCKETIO_PORT = UDP_PORT
SOCKETIO_HOST = UDP_HOST
SOCKETIO_CORS_ORIGINS = '*'  # Not used with UDP

# Data publishing
PUBLISH_RATE = 10  # Hz (fréquence d'envoi des données)

# Loop timing
LOOP_SLEEP_TIME = 0.05

# Pan/Tilt constants
PAN_TILT_BAUDRATE = 115200
PAN_TILT_SERIAL_PORT = None  # Will be set below
PAN_MIN = -1.0  # Minimum pan position
PAN_MAX = 1.0   # Maximum pan position
TILT_MIN = -1.0  # Minimum tilt position
TILT_MAX = 1.0   # Maximum tilt position
PAN_TILT_STEP_SIZE = 0.05  # Step size for discrete movement
PAN_TILT_SEND_HZ = 20  # Serial communication frequency
PAN_TILT_DEADZONE = 0.08  # Deadzone for analog stick input

# Set DEFAULT_SERIAL_PORT and PAN_TILT_SERIAL_PORT after loading ports_config
DEFAULT_SERIAL_PORT = _VESC_PORT
PAN_TILT_SERIAL_PORT = _PAN_TILT_PORT

# Camera device ID (from ports_config.py)
# None = use first available camera, or index (0, 1, ...) or mxid string
CAMERA_DEVICE_ID = _CAMERA_DEVICE_ID
