"""
Motor controller for VESC motor management.
"""

import time
from pyvesc import VESC
from .config import DEFAULT_SERIAL_PORT


class MotorController:
    """
    Controller for VESC motor with retry initialization and safe shutdown.
    """
    
    def __init__(self, serial_port=DEFAULT_SERIAL_PORT, enabled=True):
        """
        Initialize the motor controller.
        
        Args:
            serial_port: Path to the VESC serial port.
            enabled: If False, motor is disabled (mock mode) for testing without VESC.
        """
        self.serial_port = serial_port
        self.enabled = enabled
        self.motor = None
        self._initialized = False
    
    def initialize(self, max_retries=5):
        """
        Initialize the VESC motor with retry logic.
        
        Args:
            max_retries: Maximum number of initialization attempts.
            
        Returns:
            bool: True if initialization successful, False otherwise.
            
        Raises:
            RuntimeError: If initialization fails after all retries (only if enabled=True).
        """
        if not self.enabled:
            print("[Motor] Motor disabled (mock mode)")
            self._initialized = True
            return True
        
        for attempt in range(max_retries):
            try:
                self.motor = VESC(serial_port=self.serial_port)
                version = self.motor.get_firmware_version()
                if version:
                    print(f"[Motor] Firmware version: {version}")
                else:
                    print("[Motor] Warning: Firmware version is None")
                self._initialized = True
                return True
            except Exception as e:
                print(f"[Motor] Init failed (try {attempt+1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    time.sleep(1)
        
        raise RuntimeError(f"Failed to initialize VESC after {max_retries} attempts.")
    
    def set_commands(self, acceleration, steering):
        """
        Send commands to the motor: throttle (duty cycle) and steering (servo).
        
        Args:
            acceleration: Acceleration value (duty cycle, typically -1.0 to 1.0).
            steering: Steering value (raw joystick X axis, -1 to 1).
        """
        if not self.enabled or not self._initialized:
            return  # Ne rien faire si désactivé
        
        if self.motor is None:
            raise RuntimeError("Motor not initialized. Call initialize() first.")
        
        # Convert steering from [-1, 1] to [0, 1] for servo
        steer_value = (steering + 1) / 2
        self.motor.set_servo(steer_value)
        self.motor.set_duty_cycle(acceleration)
    
    def stop(self):
        """
        Stop the motor safely.
        """
        if not self.enabled:
            return  # Ne rien faire si désactivé
        
        if self.motor is not None:
            try:
                self.motor.set_rpm(0)
                self.motor.stop_heartbeat()
            except Exception as e:
                print(f"[Motor] Error during stop: {e}")
            finally:
                self._initialized = False
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
