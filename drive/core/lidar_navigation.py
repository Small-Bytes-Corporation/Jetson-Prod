"""
Advanced Navigation Algorithm - State Machine based Follow the Gap.
Refactored to allow Runtime Tuning via Ncurses.
Includes:
- Angular Sensitivity: Detects further in front, tighter on sides.
- Soft-Start for Reverse.
- Stuck Detection.
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
DEFAULT_BUBBLE_RADIUS = 0.45    # Augmenté car il sera réduit sur les côtés par le calcul
DEFAULT_CRITICAL_BUFFER = 0.25  # Augmenté pour détecter plus loin devant (0.16 + 0.25 = 0.41m)
DEFAULT_REVERSE_SPEED = -0.03 
DEFAULT_STOP_TIME = 1.5

# Angle limite pour la Safety Bubble et la détection frontale
BUBBLE_ENABLE_ANGLE = 25.0 

# Stuck Detection
STUCK_TIMEOUT = 1.5
STUCK_VARIATION_THRESHOLD = 0.05

class NavState(Enum):
    FORWARD = auto()
    BRAKING_TO_REVERSE = auto()
    REVERSING = auto()
    BRAKING_TO_FORWARD = auto()

class LidarNavigator:
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
        
        self.alpha_steering = 0.6
        self.robot_bubble_radius = DEFAULT_BUBBLE_RADIUS
        self.safety_buffer = DEFAULT_CRITICAL_BUFFER
        self.stop_wait_time = DEFAULT_STOP_TIME
        self.reverse_speed = DEFAULT_REVERSE_SPEED
        self.reverse_duration = 1.5
        self.max_lidar_dist = 3.5 # Augmenté pour voir plus loin
        
        self.field_of_view = field_of_view
        self.state = NavState.FORWARD
        self.state_timer = 0.0
        self.last_steering = 0.0
        self.last_gap_center_steering = 0.0
        self.current_reverse_power = 0.0
        self.stuck_timer = 0.0
        self.previous_min_dist = 0.0

    def _get_dynamic_critical_distance(self, angle: float) -> float:
        """
        Calcule la distance d'arrêt selon l'angle.
        Plus l'angle est grand (vers les côtés), plus la distance autorisée est courte.
        """
        base_critical = LIDAR_TO_FRONT + self.safety_buffer
        # On utilise un cosinus pour réduire la distance sur les bords du cône de 35°
        # A 0°, cos=1.0. A 35°, cos=0.81.
        angle_rad = math.radians(angle)
        return base_critical * math.cos(angle_rad)

    def compute_commands(self, lidar_scan: List[Dict]) -> Tuple[float, float]:
        current_time = time.monotonic()
        ranges = self._preprocess_scan(lidar_scan)
        
        if len(ranges) == 0:
            return 0.0, 0.0

        # Trouver l'obstacle le plus proche ET son angle
        closest_idx = np.argmin(ranges[:, 1])
        min_dist = ranges[closest_idx, 1]
        closest_angle = ranges[closest_idx, 0]
        
        acceleration = 0.0
        steering = self.last_steering

        # --- STATE MACHINE ---
        if self.state == NavState.FORWARD:
            # 1. DETECTION OBSTACLE CRITIQUE DYNAMIQUE
            # On ne vérifie le blocage que dans le cône frontal
            is_blocking = False
            if abs(closest_angle) <= BUBBLE_ENABLE_ANGLE:
                if min_dist < self._get_dynamic_critical_distance(closest_angle):
                    is_blocking = True

            if is_blocking:
                print(f"[Nav] Obstacle détecté à {closest_angle:.1f}° : {min_dist:.2f}m. Recul.")
                self.state = NavState.BRAKING_TO_REVERSE
                self.state_timer = current_time
                acceleration = 0.0
                
            # 2. DETECTION DE BLOCAGE (STUCK)
            elif abs(min_dist - self.previous_min_dist) < STUCK_VARIATION_THRESHOLD:
                if (current_time - self.stuck_timer) > STUCK_TIMEOUT:
                    self.state = NavState.BRAKING_TO_REVERSE
                    self.state_timer = current_time
                    acceleration = 0.0
            else:
                self.stuck_timer = current_time
                self.previous_min_dist = min_dist

            # 3. CONDUITE NORMALE
            if self.state == NavState.FORWARD:
                acceleration, steering = self._logic_follow_the_gap(ranges, closest_idx, min_dist)
                steering = (self.alpha_steering * steering) + ((1.0 - self.alpha_steering) * self.last_steering)
                self.last_gap_center_steering = steering

        elif self.state == NavState.BRAKING_TO_REVERSE:
            acceleration = 0.0
            self.current_reverse_power = 0.0
            if (current_time - self.state_timer) > self.stop_wait_time:
                self.state = NavState.REVERSING
                self.state_timer = current_time

        elif self.state == NavState.REVERSING:
            if self.current_reverse_power > self.reverse_speed:
                self.current_reverse_power -= 0.02
            acceleration = self.current_reverse_power
            steering = -1.0 * np.sign(self.last_gap_center_steering)
            if (current_time - self.state_timer) > self.reverse_duration:
                self.state = NavState.BRAKING_TO_FORWARD
                self.state_timer = current_time

        elif self.state == NavState.BRAKING_TO_FORWARD:
            acceleration = 0.0
            if (current_time - self.state_timer) > self.stop_wait_time:
                self.state = NavState.FORWARD
                self.last_steering = 0.0

        self.last_steering = steering
        return acceleration, steering

    def _apply_safety_bubble(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> np.ndarray:
        """
        Applique une bulle dont le rayon diminue avec l'angle.
        """
        proc_ranges = ranges.copy()
        closest_angle = ranges[closest_idx, 0]
        
        # On n'applique la bulle que si l'obstacle est dans le cône
        if abs(closest_angle) <= BUBBLE_ENABLE_ANGLE:
            if min_dist < self.max_lidar_dist:
                if min_dist < 0.1: min_dist = 0.1
                
                # RÉDUCTION DE LA SÉCURITÉ SUR LES CÔTÉS :
                # Le rayon de la bulle est multiplié par le cosinus de l'angle.
                # Plein centre (0°) -> Rayon 100%
                # Sur le côté (30°) -> Rayon ~86%
                scaling = math.cos(math.radians(closest_angle))
                effective_radius = self.robot_bubble_radius * scaling
                
                angle_radius_deg = np.degrees(np.arctan(effective_radius / min_dist))
                mask = np.abs(proc_ranges[:, 0] - closest_angle) < angle_radius_deg
                proc_ranges[mask, 1] = 0.0
            
        return proc_ranges

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

    def _logic_follow_the_gap(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> Tuple[float, float]:
        proc_ranges = self._apply_safety_bubble(ranges, closest_idx, min_dist)
        gap_start, gap_end = self._find_max_gap(proc_ranges)
        target_angle = self._find_best_point(proc_ranges, gap_start, gap_end)
        steering_raw = self._angle_to_steering(target_angle)
        speed = self._compute_speed(min_dist, closest_angle=ranges[closest_idx, 0], steer_effort=abs(steering_raw))
        return speed, steering_raw

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
        return max(-1.0, min(1.0, angle / 90.0))

    def _compute_speed(self, min_dist: float, closest_angle: float, steer_effort: float) -> float:
        """
        Vitesse adaptée : ralentit plus tôt si l'obstacle est pile en face.
        """
        crit_dist = self._get_dynamic_critical_distance(closest_angle)
        if min_dist < crit_dist: return 0.0
        
        # Ralentissement progressif vers la distance critique
        dist_factor = np.clip((min_dist - crit_dist) / self.safety_distance, 0.0, 1.0)
        turn_factor = 1.0 - (0.6 * steer_effort)
        
        target_speed = self.max_speed * dist_factor * turn_factor
        return max(self.min_speed, target_speed)
