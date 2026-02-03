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
        
        # --- ETAT INTERNE ---
        self.current_pan_speed = 0.0   # Vitesse envoyée à l'Arduino
        self.current_tilt_pos = 0.0    # Position envoyée à l'Arduino
        
        # --- MEMOIRE DU JOYSTICK ---
        # On sauvegarde la dernière valeur connue du joystick 
        # pour que le clavier puisse "retomber" dessus quand on lâche les touches.
        self.joy_pan_memory = 0.0
        
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
            
            time.sleep(2)
            
            # Reset
            print("[PanTilt] Resetting camera to center/stop...")
            for _ in range(5):
                self._send_command(0.0, 0.0)
                time.sleep(0.05)
            
            self.current_pan_speed = 0.0
            self.current_tilt_pos = 0.0
            self.joy_pan_memory = 0.0
            
            self._initialized = True
            print("[PanTilt] Initialized.")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize pan/tilt controller: {e}")
    
    def _send_command(self, pan_speed, tilt_pos):
        if not self.enabled or self.serial_conn is None:
            return
        
        try:
            msg = f"{pan_speed:.2f},{tilt_pos:.2f}\n".encode('ascii')
            self.serial_conn.write(msg)
            if self.serial_conn.in_waiting:
                self.serial_conn.read(self.serial_conn.in_waiting)
        except Exception as e:
            print(f"[PanTilt] Error sending command: {e}")
    
    def update(self, pan_delta, tilt_delta):
        """
        Gère les entrées CLAVIER (Discret : -1, 0, 1).
        Appelé en boucle par le système.
        """
        if not self._initialized:
            return
        
        # --- 1. GESTION TILT (POSITION - AXE Y) ---
        # Le clavier ajoute/enlève de l'angle
        tilt_change = tilt_delta * self.step_size
        self.current_tilt_pos += tilt_change
        self.current_tilt_pos = max(self.tilt_min, min(self.tilt_max, self.current_tilt_pos))
        
        
        # --- 2. GESTION PAN (VITESSE - AXE X) ---
        
        if pan_delta != 0:
            # CAS A : UNE TOUCHE EST ENFONCÉE
            # Le clavier est prioritaire. On impose une vitesse fixe.
            VITESSE_CLAVIER = 0.6 
            
            if pan_delta > 0:
                self.current_pan_speed = VITESSE_CLAVIER
            else:
                self.current_pan_speed = -VITESSE_CLAVIER
                
        else:
            # CAS B : AUCUNE TOUCHE CLAVIER
            # Au lieu de forcer 0.0, on utilise la valeur du JOYSTICK !
            # Si le joystick est au centre, joy_pan_memory vaut 0, donc ça s'arrête.
            # Si le joystick est poussé, ça continue de tourner.
            self.current_pan_speed = self.joy_pan_memory

        # Sécurité Limites
        self.current_pan_speed = max(self.pan_min, min(self.pan_max, self.current_pan_speed))
        
        
        # --- ENVOI ---
        now = time.monotonic()
        if now >= self.next_send:
            self._send_command(self.current_pan_speed, self.current_tilt_pos)
            self.next_send = now + self.send_interval
    
    def set_analog_position(self, pan_axis, tilt_axis):
        """
        Gère les entrées JOYSTICK (Analogique : -1.0 à 1.0).
        """
        if not self._initialized:
            return
        
        # 1. Application Deadzone
        pan = 0.0 if abs(pan_axis) < PAN_TILT_DEADZONE else pan_axis
        tilt = 0.0 if abs(tilt_axis) < PAN_TILT_DEADZONE else tilt_axis
        
        # 2. Limites
        pan = max(self.pan_min, min(self.pan_max, pan))
        tilt = max(self.tilt_min, min(self.tilt_max, tilt))
        
        # 3. MISE A JOUR TILT (Position Absolue)
        # Si le stick bouge, il prend le contrôle de la position
        if abs(tilt) > 0.0:
            self.current_tilt_pos = tilt
            # Note : Si tu lâches le stick Tilt, il revient à 0 (Horizon). 
            # C'est le comportement normal d'un joystick positionnel.
        
        # 4. MISE A JOUR PAN (Vitesse Variable)
        # On sauvegarde cette valeur en mémoire pour la méthode update()
        self.joy_pan_memory = pan
        
        # On met à jour la vitesse actuelle
        self.current_pan_speed = pan
        
        # 5. Envoi immédiat (pour réactivité joystick)
        now = time.monotonic()
        if now >= self.next_send:
            self._send_command(self.current_pan_speed, self.current_tilt_pos)
            self.next_send = now + self.send_interval
    
    def get_position(self):
        return (self.current_pan_speed, self.current_tilt_pos)
    
    def stop(self):
        if not self.enabled: return
        if self.serial_conn is not None:
            try:
                self.serial_conn.write(b"0.00,0.00\n")
                time.sleep(0.1)
                self.serial_conn.close()
            except Exception: pass
            finally:
                self._initialized = False
                self.serial_conn = None
    
    def __enter__(self):
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()