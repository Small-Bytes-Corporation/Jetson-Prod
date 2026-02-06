"""
Advanced Navigation Algorithm - State Machine based Follow the Gap.
Refactored to allow Runtime Tuning via Ncurses.
Includes Soft-Start for Reverse & Stuck Detection.
"""

import math
import time
import numpy as np
from enum import Enum, auto
from typing import List, Dict, Tuple, Optional
from .config import (
    AUTONOMOUS_MAX_SPEED, AUTONOMOUS_MIN_SPEED, AUTONOMOUS_SAFETY_DISTANCE,
    AUTONOMOUS_FIELD_OF_VIEW, AUTONOMOUS_LOOKAHEAD_DISTANCE, AUTONOMOUS_GAP_THRESHOLD
)

# Physical dimensions (fixed)
LIDAR_TO_FRONT = 0.16
LIDAR_TO_REAR = 0.04
LIDAR_ROTATION_OFFSET = 90.0

# --- Default Tuning Values ---
DEFAULT_BUBBLE_RADIUS = 0.4
DEFAULT_CRITICAL_BUFFER = 0.15 
DEFAULT_REVERSE_SPEED = -0.22  # Vitesse suffisante pour vaincre le cogging
DEFAULT_STOP_TIME = 2.0        # Temps d'arrêt pour protéger le VESC

# Angle limite pour la Safety Bubble (seulement devant)
BUBBLE_ENABLE_ANGLE = 30.0 

# Paramètres de détection de blocage (Stuck Detection)
STUCK_TIMEOUT = 2.0            # Temps sans mouvement avant de déclencher la marche arrière
STUCK_VARIATION_THRESHOLD = 0.05 # 5cm : Si le lidar varie moins que ça, on est considéré immobile

class NavState(Enum):
    FORWARD = auto()
    BRAKING_TO_REVERSE = auto()
    REVERSING = auto()
    BRAKING_TO_FORWARD = auto()

class LidarNavigator:
    """
    Navigation controller with mutable parameters for runtime tuning.
    """
    
    def __init__(
        self,
        max_speed: float = AUTONOMOUS_MAX_SPEED,
        min_speed: float = AUTONOMOUS_MIN_SPEED,
        safety_distance: float = AUTONOMOUS_SAFETY_DISTANCE,
        field_of_view: Tuple[float, float] = AUTONOMOUS_FIELD_OF_VIEW,
        lookahead_distance: float = AUTONOMOUS_LOOKAHEAD_DISTANCE,
        gap_threshold: float = AUTONOMOUS_GAP_THRESHOLD,
    ):
        # --- Tunable Parameters ---
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.safety_distance = safety_distance
        self.lookahead_distance = lookahead_distance
        self.gap_threshold = gap_threshold
        
        # Advanced tuning
        self.alpha_steering = 0.6
        self.robot_bubble_radius = DEFAULT_BUBBLE_RADIUS
        self.safety_buffer = DEFAULT_CRITICAL_BUFFER
        self.stop_wait_time = DEFAULT_STOP_TIME
        self.reverse_speed = DEFAULT_REVERSE_SPEED
        self.reverse_duration = 1.5
        self.max_lidar_dist = 3.0
        
        self.field_of_view = field_of_view
        
        # State Machine variables
        self.state = NavState.FORWARD
        self.state_timer = 0.0
        
        # Internal memory
        self.last_steering = 0.0
        self.last_gap_center_steering = 0.0
        self.current_reverse_power = 0.0
        
        # Stuck Detection Memory
        self.stuck_timer = 0.0
        self.previous_min_dist = 0.0

    @property
    def critical_distance(self):
        """Dynamic calculation of critical distance."""
        return LIDAR_TO_FRONT + self.safety_buffer

    def compute_commands(self, lidar_scan: List[Dict]) -> Tuple[float, float]:
        current_time = time.monotonic()
        ranges = self._preprocess_scan(lidar_scan)
        
        if len(ranges) == 0:
            return 0.0, 0.0

        closest_idx = np.argmin(ranges[:, 1])
        min_dist = ranges[closest_idx, 1]
        
        acceleration = 0.0
        steering = self.last_steering

        # --- STATE MACHINE ---
        
        if self.state == NavState.FORWARD:
            
            # --- 1. DETECTION OBSTACLE CRITIQUE ---
            if min_dist < self.critical_distance:
                print(f"[Nav] Obstacle critique ({min_dist:.2f}m). Recul.")
                self.state = NavState.BRAKING_TO_REVERSE
                self.state_timer = current_time
                acceleration = 0.0
                
            # --- 2. DETECTION DE BLOCAGE (PATINAGE) ---
            # Si la distance minimale ne change pas assez alors qu'on essaye d'avancer
            elif abs(min_dist - self.previous_min_dist) < STUCK_VARIATION_THRESHOLD:
                # Si ça fait plus de 2 secondes que ça ne bouge pas
                if (current_time - self.stuck_timer) > STUCK_TIMEOUT:
                    print(f"[Nav] Bloqué détecté (Lidar statique). Tentative de dégagement.")
                    self.state = NavState.BRAKING_TO_REVERSE
                    self.state_timer = current_time
                    acceleration = 0.0
                    # On reset le timer pour ne pas re-déclencher immédiatement après
                    self.stuck_timer = current_time 
            else:
                # Ça bouge, on reset le timer de blocage
                self.stuck_timer = current_time
                self.previous_min_dist = min_dist

            # --- 3. CONDUITE NORMALE ---
            if self.state == NavState.FORWARD:
                acceleration, steering = self._logic_follow_the_gap(ranges, closest_idx, min_dist)
                
                # Apply smoothing
                steering = (self.alpha_steering * steering) + \
                           ((1.0 - self.alpha_steering) * self.last_steering)
                self.last_gap_center_steering = steering

        elif self.state == NavState.BRAKING_TO_REVERSE:
            acceleration = 0.0
            steering = self.last_steering
            self.current_reverse_power = 0.0
            
            if (current_time - self.state_timer) > self.stop_wait_time:
                self.state = NavState.REVERSING
                self.state_timer = current_time

        elif self.state == NavState.REVERSING:
            # SOFT START RAMP
            ramp_speed = 0.02
            if self.current_reverse_power > self.reverse_speed:
                self.current_reverse_power -= ramp_speed
                if self.current_reverse_power < self.reverse_speed:
                    self.current_reverse_power = self.reverse_speed
            
            acceleration = self.current_reverse_power
            
            # Invert steering logic
            steering = -1.0 * np.sign(self.last_gap_center_steering)
            if abs(steering) < 0.1: steering = 0.0
            
            if (current_time - self.state_timer) > self.reverse_duration:
                self.state = NavState.BRAKING_TO_FORWARD
                self.state_timer = current_time

        elif self.state == NavState.BRAKING_TO_FORWARD:
            acceleration = 0.0
            steering = self.last_steering
            if (current_time - self.state_timer) > self.stop_wait_time:
                self.state = NavState.FORWARD
                self.last_steering = 0.0
                # Reset stuck detection variables when restarting forward
                self.stuck_timer = current_time
                self.previous_min_dist = min_dist

        self.last_steering = steering
        return acceleration, steering

    def _logic_follow_the_gap(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> Tuple[float, float]:
        proc_ranges = self._apply_safety_bubble(ranges, closest_idx, min_dist)
        gap_start, gap_end = self._find_max_gap(proc_ranges)
        target_angle = self._find_best_point(proc_ranges, gap_start, gap_end)
        steering_raw = self._angle_to_steering(target_angle)
        speed = self._compute_speed(min_dist, abs(steering_raw))
        return speed, steering_raw

    def _preprocess_scan(self, scan: List[Dict]) -> np.ndarray:
        data = []
        for p in scan:
            angle = p['angle'] + LIDAR_ROTATION_OFFSET
            if angle > 180: angle -= 360
            if angle <= -180: angle += 360
            
            if self.field_of_view[0] <= angle <= self.field_of_view[1]:
                dist = p['distance']
                if dist > self.max_lidar_dist: dist = self.max_lidar_dist
                if dist < 0.1: dist = 0.0
                data.append([angle, dist])
        
        if not data: return np.array([])
        arr = np.array(data)
        return arr[arr[:, 0].argsort()]

    def _apply_safety_bubble(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> np.ndarray:
        proc_ranges = ranges.copy()
        if min_dist < self.max_lidar_dist:
            if min_dist < 0.1: min_dist = 0.1
            
            closest_angle = ranges[closest_idx, 0]
            
            # Apply bubble only if obstacle is generally in front (+/- 30 deg)
            if abs(closest_angle) <= BUBBLE_ENABLE_ANGLE:
                angle_radius_rad = np.arctan(self.robot_bubble_radius / min_dist)
                angle_radius_deg = np.degrees(angle_radius_rad)
                
                mask = np.abs(proc_ranges[:, 0] - closest_angle) < angle_radius_deg
                proc_ranges[mask, 1] = 0.0
            
        return proc_ranges

    def _find_max_gap(self, ranges: np.ndarray) -> Tuple[int, int]:
        mask = ranges[:, 1] > self.gap_threshold
        padded = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0] - 1
        if len(starts) == 0: return 0, len(ranges) - 1
        lengths = ends - starts
        longest_idx = np.argmax(lengths)
        return starts[longest_idx], ends[longest_idx]

    def _find_best_point(self, ranges: np.ndarray, start: int, end: int) -> float:
        gap_points = ranges[start:end+1]
        if len(gap_points) == 0: return 0.0
        return gap_points[len(gap_points)//2, 0]

    def _angle_to_steering(self, angle: float) -> float:
        steer = angle / 90.0
        return max(-1.0, min(1.0, steer))

    def _compute_speed(self, closest_dist: float, steer_effort: float) -> float:
        if closest_dist < self.critical_distance: return 0.0
        dist_factor = np.clip((closest_dist - self.critical_distance) / (self.safety_distance), 0.0, 1.0)
        turn_factor = 1.0 - (0.6 * steer_effort)
        target_speed = self.max_speed * dist_factor * turn_factor
        return max(self.min_speed, target_speed)
