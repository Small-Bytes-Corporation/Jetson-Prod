"""
Configuration constants for the robocar control system.
"""

import platform
import glob
import os

# Load environment variables from .env file if available
try:
    from dotenv import load_dotenv
    # Load .env file from project root (parent of drive/core/)
    _env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '.env')
    load_dotenv(_env_path)
except ImportError:
    # python-dotenv not installed, skip loading .env
    pass

# Motor control constants
MAX_SPEED = 0.15
DELTA_ACC = 0.003
DELTA_BRAKE = 0.006
# DEFAULT_SERIAL_PORT will be set after loading ports_config.py
DEFAULT_SERIAL_PORT = None  # Will be set below

# Camera constants
CAM_WIDTH = 320
CAM_HEIGHT = 180
CAM_FPS = 30

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
        _RTK_PORT = getattr(ports_config, 'RTK_PORT', '/dev/ttyUSB1')
        _PAN_TILT_PORT = getattr(ports_config, 'PAN_TILT_PORT', '/dev/ttyACM2')
        _CAMERA_DEVICE_ID = getattr(ports_config, 'CAMERA_DEVICE_ID', None)
    else:
        # Fallback to defaults if ports_config.py doesn't exist
        _VESC_PORT = '/dev/ttyACM1'
        _LIDAR_PORT = '/dev/ttyUSB0'
        _RTK_PORT = '/dev/ttyUSB1'
        _PAN_TILT_PORT = '/dev/ttyACM2'
        _CAMERA_DEVICE_ID = None
except Exception:
    # Fallback to defaults if import fails
    _VESC_PORT = '/dev/ttyACM1'
    _LIDAR_PORT = '/dev/ttyUSB0'
    _RTK_PORT = '/dev/ttyUSB1'
    _PAN_TILT_PORT = '/dev/ttyACM2'
    _CAMERA_DEVICE_ID = None

# Lidar constants (default from ports_config.py)
DEFAULT_LIDAR_PORT = _LIDAR_PORT

LIDAR_BAUDRATE = 230400  # À vérifier selon documentation D500

# Socket.io constants
SOCKETIO_PORT = 3000
SOCKETIO_HOST = '0.0.0.0'
SOCKETIO_CORS_ORIGINS = '*'  # À restreindre en production

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

# RTK GNSS (Point One / Quectel LG69T) constants (default from ports_config.py)
DEFAULT_RTK_SERIAL_PORT = _RTK_PORT
RTK_BAUDRATE = 460800
# TCP port for p1-runner output (when using init_rtk --tcp). main.py connects here for valid RTK fix.
RTK_TCP_DEFAULT_PORT = 30201

# Set DEFAULT_SERIAL_PORT and PAN_TILT_SERIAL_PORT after loading ports_config
DEFAULT_SERIAL_PORT = _VESC_PORT
PAN_TILT_SERIAL_PORT = _PAN_TILT_PORT

# Camera device ID (from ports_config.py)
# None = use first available camera, or index (0, 1, ...) or mxid string
CAMERA_DEVICE_ID = _CAMERA_DEVICE_ID

# RTK GNSS Polaris configuration
# Load from environment variables if available, otherwise use defaults
POLARIS_API_KEY = os.getenv('POLARIS_API_KEY', 'be05a69030d24ec883c3a704d48dcb50')
POLARIS_NTRIP_HOST = os.getenv('POLARIS_NTRIP_HOST', 'truertk.pointonenav.com')  # ou virtualrtk.pointonenav.com
POLARIS_NTRIP_PORT = int(os.getenv('POLARIS_NTRIP_PORT', '2102'))  # TLS encrypted (recommandé) ou 2101 pour plaintext
POLARIS_NTRIP_MOUNT_POINT = os.getenv('POLARIS_NTRIP_MOUNT_POINT', 'POLARIS')  # ITRF2014 global datum
