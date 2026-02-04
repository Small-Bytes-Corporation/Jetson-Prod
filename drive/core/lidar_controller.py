"""
Lidar controller for LDROBOT D500 LiDarKit.
Optimized for D500/LD19 Protocol (Header 0x54 0x2C).
"""

import time
import os
import serial
import struct
import math
from .config import DEFAULT_LIDAR_PORT, LIDAR_BAUDRATE, _get_available_serial_ports, IS_MAC

class LidarController:
    def __init__(self, serial_port=DEFAULT_LIDAR_PORT, baudrate=LIDAR_BAUDRATE, enabled=True, debug=False):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.enabled = enabled
        self.debug = debug
        self.serial_conn = None
        self._initialized = False
        
        # Buffers de données
        self.last_scan = []           
        self.current_scan_buffer = [] 
        self.last_angle = 0.0
        self._buffer = bytearray()

    def initialize(self):
        if not self.enabled:
            if self.debug:
                print("[Lidar] Disabled (mock mode)")
            self._initialized = True
            return True
        
        try:
            # Ouverture du port série
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self._initialized = True
            if self.debug:
                print(f"[Lidar] Initialized on {self.serial_port} @ {self.baudrate}")
            return True
        except Exception as e:
            if self.debug:
                print(f"[Lidar] Init failed: {e}")
            return False

    def get_scan(self):
        """
        Lit le port série et retourne un scan complet (une rotation).
        """
        if not self._initialized or self.serial_conn is None:
            return None

        try:
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self._buffer.extend(data)
                
                # Tant qu'on a assez de données pour un paquet (47 bytes)
                while len(self._buffer) >= 47:
                    # 1. Recherche Header 0x54
                    if self._buffer[0] != 0x54:
                        self._buffer.pop(0)
                        continue
                    
                    # 2. Vérification VerLen 0x2C
                    if self._buffer[1] != 0x2C:
                        self._buffer.pop(0)
                        continue

                    # 3. Extraction paquet
                    packet = self._buffer[:47]
                    self._buffer = self._buffer[47:]
                    
                    # 4. Traitement
                    self._process_packet(packet)

                # Si un tour est fini, on le retourne
                if self.last_scan:
                    scan_to_return = self.last_scan
                    self.last_scan = [] 
                    return scan_to_return
            
            return None

        except Exception as e:
            if self.debug:
                print(f"[Lidar] Error: {e}")
            self._buffer = bytearray() # Reset buffer on error
            return None

    def _process_packet(self, data):
        """
        Décode un paquet LD19/D500 (47 bytes).
        Structure:
        Byte 0: 0x54 (Header)
        Byte 1: 0x2C (VerLen)
        Byte 2-3: Speed
        Byte 4-5: Start Angle
        Byte 6-41: Data (12 points * 3 bytes)
        Byte 42-43: End Angle
        Byte 44-45: Timestamp
        Byte 46: CRC
        """
        try:
            # CORRECTION OFFSETS ICI
            # Start Angle est à l'index 4 (pas 2 !)
            start_angle = struct.unpack_from('<H', data, 4)[0] / 100.0
            # End Angle est à l'index 42 (pas 40 !)
            end_angle = struct.unpack_from('<H', data, 42)[0] / 100.0

            if end_angle < start_angle:
                end_angle += 360.0

            step = (end_angle - start_angle) / 11.0 if (end_angle - start_angle) != 0 else 0

            # Lecture des 12 points
            for i in range(12):
                # Les données commencent à l'index 6
                base = 6 + (i * 3) 
                
                distance_mm = struct.unpack_from('<H', data, base)[0]
                confidence = data[base + 2]
                
                distance_m = distance_mm / 1000.0

                # Filtrage
                if 0.15 < distance_m < 12.0 and confidence > 100:
                    angle = start_angle + (step * i)
                    if angle >= 360.0: angle -= 360.0
                    
                    self.current_scan_buffer.append({
                        "angle": angle,       
                        "distance": distance_m, 
                        "intensity": confidence
                    })

            # Fin de tour
            if start_angle < (self.last_angle - 50.0):
                self.last_scan = list(self.current_scan_buffer)
                self.current_scan_buffer = []

            self.last_angle = start_angle

        except Exception as e:
            if self.debug:
                print(f"[Lidar] Parse error: {e}")

    def is_available(self):
        return self._initialized

    def stop(self):
        if self.serial_conn:
            self.serial_conn.close()
        self._initialized = False
