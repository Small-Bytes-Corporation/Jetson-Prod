"""
Motor controller for VESC motor management.
"""

import time
import threading
from pyvesc import VESC
from .config import DEFAULT_SERIAL_PORT


class MotorController:
    """
    Controller for VESC motor with retry initialization and safe shutdown.
    """
    
    def __init__(self, serial_port=DEFAULT_SERIAL_PORT, enabled=True, error_callback=None):
        """
        Initialize the motor controller.
        
        Args:
            serial_port: Path to the VESC serial port.
            enabled: If False, motor is disabled (mock mode) for testing without VESC.
            error_callback: Optional callback function(message: str) for error logging.
        """
        self.serial_port = serial_port
        self.enabled = enabled
        self.motor = None
        self._initialized = False
        self.error_callback = error_callback
    
    def initialize(self, max_retries=5, timeout=3):
        """
        Initialize the VESC motor with retry logic and timeout.
        
        Args:
            max_retries: Maximum number of initialization attempts.
            timeout: Timeout in seconds for each initialization attempt.
            
        Returns:
            bool: True if initialization successful, False otherwise.
            
        Raises:
            RuntimeError: If initialization fails after all retries (only if enabled=True).
        """
        if not self.enabled:
            if self.error_callback:
                self.error_callback("[Motor] Motor disabled (mock mode)")
            else:
                print("[Motor] Motor disabled (mock mode)")
            self._initialized = True
            return True
        
        def init_vesc():
            """Initialize VESC in a separate thread."""
            try:
                motor = VESC(serial_port=self.serial_port)
                version = motor.get_firmware_version()
                return motor, version
            except Exception as e:
                raise e
        
        for attempt in range(max_retries):
            try:
                result = [None]
                exception = [None]
                
                def target():
                    try:
                        motor, version = init_vesc()
                        result[0] = (motor, version)
                    except Exception as e:
                        exception[0] = e
                
                thread = threading.Thread(target=target)
                thread.daemon = True
                thread.start()
                thread.join(timeout=timeout)
                
                if thread.is_alive():
                    msg = f"[Motor] Init timeout (try {attempt+1}/{max_retries}): VESC connection took longer than {timeout}s"
                    if self.error_callback:
                        self.error_callback(msg)
                    else:
                        print(msg)
                    if attempt < max_retries - 1:
                        time.sleep(1)
                    continue
                
                if exception[0]:
                    raise exception[0]
                
                if result[0]:
                    self.motor, version = result[0]
                    if version:
                        msg = f"[Motor] Firmware version: {version}"
                        if self.error_callback:
                            self.error_callback(msg)
                        else:
                            print(msg)
                    else:
                        msg = "[Motor] Warning: Firmware version is None"
                        if self.error_callback:
                            self.error_callback(msg)
                        else:
                            print(msg)
                    self._initialized = True
                    return True
                    
            except Exception as e:
                msg = f"[Motor] Init failed (try {attempt+1}/{max_retries}): {e}"
                if self.error_callback:
                    self.error_callback(msg)
                else:
                    print(msg)
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
                msg = f"[Motor] Error during stop: {e}"
                if self.error_callback:
                    self.error_callback(msg)
                else:
                    print(msg)
            finally:
                self._initialized = False
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
