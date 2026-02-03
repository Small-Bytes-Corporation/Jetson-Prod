"""
Pan/Tilt controller for camera mount servo control via serial communication.
"""

import time
import serial
from .config import (
    PAN_TILT_SERIAL_PORT, PAN_TILT_BAUDRATE,
    PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX,
    PAN_TILT_STEP_SIZE, PAN_TILT_SEND_HZ
)


class PanTiltController:
    """
    Controller for pan/tilt camera mount with discrete movement and position limits.
    """
    
    def __init__(self, serial_port=PAN_TILT_SERIAL_PORT, enabled=True,
                 pan_min=PAN_MIN, pan_max=PAN_MAX,
                 tilt_min=TILT_MIN, tilt_max=TILT_MAX,
                 step_size=PAN_TILT_STEP_SIZE):
        """
        Initialize the pan/tilt controller.
        
        Args:
            serial_port: Path to serial port for Arduino communication.
            enabled: If False, controller is disabled (mock mode).
            pan_min: Minimum pan position (default: -1.0).
            pan_max: Maximum pan position (default: 1.0).
            tilt_min: Minimum tilt position (default: -1.0).
            tilt_max: Maximum tilt position (default: 1.0).
            step_size: Step size for discrete movement (default: 0.05).
        """
        self.serial_port = serial_port
        self.enabled = enabled
        self.pan_min = pan_min
        self.pan_max = pan_max
        self.tilt_min = tilt_min
        self.tilt_max = tilt_max
        self.step_size = step_size
        
        self.serial_conn = None
        self.pan_position = 0.0
        self.tilt_position = 0.0
        self._initialized = False
        
        # Rate limiting for serial communication
        self.next_send = 0.0
        self.send_interval = 1.0 / PAN_TILT_SEND_HZ
    
    def initialize(self):
        """
        Initialize serial connection and reset pan/tilt to center position.
        
        Raises:
            RuntimeError: If serial connection fails.
        """
        if not self.enabled:
            print("[PanTilt] Controller disabled (mock mode)")
            self._initialized = True
            return
        
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=PAN_TILT_BAUDRATE,
                timeout=0,
                write_timeout=0
            )
            print(f"[PanTilt] Serial connection opened on {self.serial_port}")
            
            # Wait for Arduino reboot
            time.sleep(2)
            
            # Reset to center position (0, 0)
            print("[PanTilt] Resetting camera position to center...")
            for _ in range(5):
                self._send_position(0.0, 0.0)
                time.sleep(0.05)
            
            self.pan_position = 0.0
            self.tilt_position = 0.0
            self._initialized = True
            print("[PanTilt] Initialized and reset to center position")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize pan/tilt controller: {e}")
    
    def _send_position(self, pan, tilt):
        """
        Send pan/tilt position to Arduino via serial.
        
        Args:
            pan: Pan position value.
            tilt: Tilt position value.
        """
        if not self.enabled or self.serial_conn is None:
            return
        
        try:
            msg = f"{pan:.2f},{tilt:.2f}\n".encode('ascii')
            self.serial_conn.write(msg)
            # Clear any pending input
            if self.serial_conn.in_waiting:
                self.serial_conn.read(self.serial_conn.in_waiting)
        except Exception as e:
            print(f"[PanTilt] Error sending position: {e}")
    
    def update(self, pan_delta, tilt_delta):
        """
        Update pan/tilt position with discrete deltas, applying limits.
        
        Args:
            pan_delta: Change in pan position (will be multiplied by step_size).
            tilt_delta: Change in tilt position (will be multiplied by step_size).
        """
        if not self._initialized:
            return
        
        # Apply step size to deltas
        pan_change = pan_delta * self.step_size
        tilt_change = tilt_delta * self.step_size
        
        # Calculate new positions
        new_pan = self.pan_position + pan_change
        new_tilt = self.tilt_position + tilt_change
        
        # Apply limits
        new_pan = max(self.pan_min, min(self.pan_max, new_pan))
        new_tilt = max(self.tilt_min, min(self.tilt_max, new_tilt))
        
        # Only send if position changed or rate limit allows
        now = time.monotonic()
        if (new_pan != self.pan_position or new_tilt != self.tilt_position) and now >= self.next_send:
            self.pan_position = new_pan
            self.tilt_position = new_tilt
            self._send_position(self.pan_position, self.tilt_position)
            self.next_send = now + self.send_interval
    
    def get_position(self):
        """
        Get current pan/tilt position.
        
        Returns:
            tuple: (pan, tilt) current position.
        """
        return (self.pan_position, self.tilt_position)
    
    def stop(self):
        """Stop the controller and close serial connection."""
        if not self.enabled:
            return
        
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
                print("[PanTilt] Serial connection closed")
            except Exception as e:
                print(f"[PanTilt] Error during stop: {e}")
            finally:
                self._initialized = False
                self.serial_conn = None
    
    def __enter__(self):
        """Context manager entry."""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
