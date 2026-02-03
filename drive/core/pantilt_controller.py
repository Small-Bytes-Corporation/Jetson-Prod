"""
Pan/Tilt controller for camera mount servo control via serial communication.
"""

import time
import serial
from .config import (
    PAN_TILT_SERIAL_PORT, PAN_TILT_BAUDRATE,
    PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX,
    PAN_TILT_STEP_SIZE, PAN_TILT_SEND_HZ, PAN_TILT_DEADZONE
)


class PanTiltController:
    """
    Controller for pan/tilt camera mount.
    PAN (X) is controlled by SPEED (Continuous Rotation Servo).
    TILT (Y) is controlled by POSITION (Standard Servo).
    """
    
    def __init__(self, serial_port=PAN_TILT_SERIAL_PORT, enabled=True,
                 pan_min=PAN_MIN, pan_max=PAN_MAX,
                 tilt_min=TILT_MIN, tilt_max=TILT_MAX,
                 step_size=PAN_TILT_STEP_SIZE):
        
        self.serial_port = serial_port
        self.enabled = enabled
        self.pan_min = pan_min
        self.pan_max = pan_max
        self.tilt_min = tilt_min
        self.tilt_max = tilt_max
        self.step_size = step_size
        
        self.serial_conn = None
        
        # State variables
        self.current_pan_speed = 0.0   # Vitesse actuelle du Pan (-1 à 1)
        self.current_tilt_pos = 0.0    # Position actuelle du Tilt (-1 à 1)
        
        self._initialized = False
        
        # Rate limiting
        self.next_send = 0.0
        self.send_interval = 1.0 / PAN_TILT_SEND_HZ
    
    def initialize(self):
        """
        Initialize serial connection and stop motors.
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
            
            # Reset: Pan STOP (0 speed), Tilt HORIZON (0 position)
            print("[PanTilt] Resetting camera to center/stop...")
            for _ in range(5):
                self._send_command(0.0, 0.0)
                time.sleep(0.05)
            
            self.current_pan_speed = 0.0
            self.current_tilt_pos = 0.0
            self._initialized = True
            print("[PanTilt] Initialized.")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize pan/tilt controller: {e}")
    
    def _send_command(self, pan_speed, tilt_pos):
        """
        Send command to Arduino.
        Args:
            pan_speed: Speed for continuous servo (-1.0 to 1.0).
            tilt_pos: Angle for positional servo (-1.0 to 1.0).
        """
        if not self.enabled or self.serial_conn is None:
            return
        
        try:
            # Format: "PAN_SPEED,TILT_POS\n"
            msg = f"{pan_speed:.2f},{tilt_pos:.2f}\n".encode('ascii')
            self.serial_conn.write(msg)
            
            # Clear input buffer to avoid lag
            if self.serial_conn.in_waiting:
                self.serial_conn.read(self.serial_conn.in_waiting)
        except Exception as e:
            print(f"[PanTilt] Error sending command: {e}")
    
    def update(self, pan_delta, tilt_delta):
        """
        Update pan/tilt from discrete inputs (Arrow keys).
        
        Args:
            pan_delta:  -1 (Left), 0 (None), 1 (Right).
            tilt_delta: -1 (Down), 0 (None), 1 (Up).
        """
        if not self._initialized:
            return
        
        # --- GESTION DU TILT (POSITION - AXE Y) ---
        # On AJOUTE la variation à la position actuelle (pour que la tête monte/descende et reste là)
        tilt_change = tilt_delta * self.step_size
        self.current_tilt_pos += tilt_change
        
        # Limites Tilt
        self.current_tilt_pos = max(self.tilt_min, min(self.tilt_max, self.current_tilt_pos))
        
        
        # --- GESTION DU PAN (VITESSE - AXE X) ---
        # ICI ON CHANGE LA LOGIQUE :
        # Au lieu d'ajouter (+/-), on DÉFINIT la vitesse directement.
        
        if pan_delta != 0:
            # Si on appuie sur une flèche, on met une vitesse fixe (ex: 50% de la vitesse max)
            # pan_delta est soit 1.0 soit -1.0 (ou 0.0)
            # On utilise 0.5 pour ne pas tourner trop vite avec le clavier, ou 1.0 pour fond.
            VITESSE_CLAVIER = 0.6 
            
            if pan_delta > 0:
                self.current_pan_speed = VITESSE_CLAVIER  # Tourne à Droite
            else:
                self.current_pan_speed = -VITESSE_CLAVIER # Tourne à Gauche
        else:
            # Si aucune touche Pan n'est enfoncée, ON STOPPE NET
            self.current_pan_speed = 0.0

        # Limites Pan (Sécurité)
        self.current_pan_speed = max(self.pan_min, min(self.pan_max, self.current_pan_speed))
        
        
        # --- ENVOI A L'ARDUINO ---
        now = time.monotonic()
        if now >= self.next_send:
            self._send_command(self.current_pan_speed, self.current_tilt_pos)
            self.next_send = now + self.send_interval
    def set_analog_position(self, pan_axis, tilt_axis):
        """
        Set directly from Joystick.
        
        Args:
            pan_axis: Joystick X value (-1 to 1). Maps to SPEED.
            tilt_axis: Joystick Y value (-1 to 1). Maps to POSITION.
        """
        if not self._initialized:
            return
        
        # Apply deadzone
        pan = 0.0 if abs(pan_axis) < PAN_TILT_DEADZONE else pan_axis
        tilt = 0.0 if abs(tilt_axis) < PAN_TILT_DEADZONE else tilt_axis
        
        # Limits
        pan = max(self.pan_min, min(self.pan_max, pan))
        tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        
        # Update internal state so get_position() is accurate
        self.current_pan_speed = pan
        self.current_tilt_pos = tilt
        
        # Always send (to ensure smooth movement and immediate stop)
        now = time.monotonic()
        if now >= self.next_send:
            self._send_command(self.current_pan_speed, self.current_tilt_pos)
            self.next_send = now + self.send_interval
    
    def get_position(self):
        """
        Returns (current_pan_speed, current_tilt_position).
        """
        return (self.current_pan_speed, self.current_tilt_pos)
    
    def stop(self):
        """Stop motors and close connection."""
        if not self.enabled:
            return
        
        if self.serial_conn is not None:
            try:
                # Force Stop command before closing
                self.serial_conn.write(b"0.00,0.00\n")
                time.sleep(0.1)
                self.serial_conn.close()
                print("[PanTilt] Serial connection closed")
            except Exception as e:
                print(f"[PanTilt] Error during stop: {e}")
            finally:
                self._initialized = False
                self.serial_conn = None
    
    def __enter__(self):
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()