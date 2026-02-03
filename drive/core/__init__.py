"""
Core modules for the robocar control system.
"""

import importlib

__all__ = [
    'MotorController',
    'JoystickController',
    'ThrottleController',
    'CameraController',
    'LidarController',
    'PanTiltController',
    'SocketServer',
    'DataPublisher',
    'RTKController',
]

# Lazy-load mapping: name -> submodule name (without .)
_MODULES = {
    'MotorController': 'motor_controller',
    'JoystickController': 'joystick_controller',
    'ThrottleController': 'throttle_controller',
    'CameraController': 'camera_controller',
    'LidarController': 'lidar_controller',
    'PanTiltController': 'pantilt_controller',
    'SocketServer': 'socket_server',
    'DataPublisher': 'data_publisher',
    'RTKController': 'rtk_controller',
}


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    if name not in _MODULES:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    mod = importlib.import_module(f".{_MODULES[name]}", __name__)
    return getattr(mod, name)
