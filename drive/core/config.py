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
DEFAULT_SERIAL_PORT = '/dev/ttyACM0'

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


def _detect_lidar_port():
    """
    Detect the default lidar port based on the platform.
    
    Returns:
        str: Default lidar port path.
    """
    available_ports = _get_available_serial_ports()
    
    if available_ports:
        # Return the first available port
        return available_ports[0]
    
    # If no port found, return platform-specific default
    if IS_MAC:
        return '/dev/tty.usbserial-0001'  # Common Mac pattern (may need manual specification)
    else:
        return '/dev/ttyUSB0'  # Standard Linux/Jetson pattern


# Lidar constants
DEFAULT_LIDAR_PORT = _detect_lidar_port()

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
PAN_TILT_SERIAL_PORT = '/dev/ttyACM0'  # Default, can be overridden via CLI
PAN_MIN = -1.0  # Minimum pan position
PAN_MAX = 1.0   # Maximum pan position
TILT_MIN = -1.0  # Minimum tilt position
TILT_MAX = 1.0   # Maximum tilt position
PAN_TILT_STEP_SIZE = 0.05  # Step size for discrete movement
PAN_TILT_SEND_HZ = 20  # Serial communication frequency
PAN_TILT_DEADZONE = 0.08  # Deadzone for analog stick input
