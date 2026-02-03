import sys
import time
import os
from enum import IntEnum
import pygame
import serial

os.environ["SDL_VIDEODRIVER"] = "dummy"

# --- CONFIGURATION ---
BAUDRATE = 115200
PORT_DEFAULT = "/dev/ttyACM0" 
SEND_HZ = 20
DEADZONE = 0.08

class Axis(IntEnum):
    # --- CORRECTION ICI ---
    # Pour le STICK DROIT sur Logitech F710 (Mode X) :
    PAN_AXIS = 3   # Axe X (Gauche/Droite) -> Doit contrôler le PAN
    TILT_AXIS = 4  # Axe Y (Haut/Bas) -> Doit contrôler le TILT
    
    # NOTE : Si ça ne marche pas, essaie le Stick GAUCHE :
    # PAN_AXIS = 0
    # TILT_AXIS = 1

def init_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("Pas de manette")
    js = pygame.joystick.Joystick(0)
    js.init()
    return js

def open_serial(port):
    return serial.Serial(port=port, baudrate=BAUDRATE, timeout=0, write_timeout=0)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT_DEFAULT
    
    try:
        js = init_joystick()
        print(f"🎮 Manette : {js.get_name()}")
        ser = open_serial(port)
        print(f"✅ Série ouvert sur {port}")
        time.sleep(2) # Attente reboot Arduino
    except Exception as e:
        print(f"Erreur : {e}")
        return 1

    print("♻️  Reset de la position caméra à 0...")
    # Reset forcé au démarrage
    for _ in range(5):
        ser.write(b"0.00,0.00\n")
        time.sleep(0.05)
    
    print("🚀 Prêt ! (CTRL+C pour quitter)")
    print(f"ℹ️  Axes configurés -> Pan(X): {Axis.PAN_AXIS} | Tilt(Y): {Axis.TILT_AXIS}")
    
    prev_x, prev_y = 0.0, 0.0
    next_send = time.monotonic()
    
    try:
        while True:
            pygame.event.pump()
            
            # Lecture des axes corrigés
            raw_x = js.get_axis(Axis.PAN_AXIS)
            raw_y = js.get_axis(Axis.TILT_AXIS)

            # Deadzone
            x = 0.0 if abs(raw_x) < DEADZONE else raw_x
            y = 0.0 if abs(raw_y) < DEADZONE else raw_y

            # Gestion Fréquence envoi
            now = time.monotonic()
            if now >= next_send:
                next_send = now + (1.0/SEND_HZ)
                
                # Formatage message : X pour Pan, Y pour Tilt
                msg = f"{x:.2f},{y:.2f}\n".encode('ascii')
                try:
                    ser.write(msg)
                    if ser.in_waiting: ser.read(ser.in_waiting)
                except Exception:
                    pass 

                # Affichage
                sys.stdout.write(f"\rPan(X): {x:+.2f} | Tilt(Y): {y:+.2f}   ")
                sys.stdout.flush()

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nArrêt.")
        if ser: ser.close()

if __name__ == "__main__":
    sys.exit(main())(depthai)