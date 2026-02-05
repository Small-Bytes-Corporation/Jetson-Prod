"""
Advanced Navigation Algorithm - State Machine based Follow the Gap.
Refactored to include physical robot dimensions and motor protection.

Physical Constraints:
- Lidar is mounted 4cm from the rear and 16cm from the front.
- Lidar is rotated 90 degrees relative to robot frame.
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

# --- Physical Dimensions ---
LIDAR_TO_FRONT = 0.16          # 16 cm du lidar au nez de la voiture
LIDAR_TO_REAR = 0.04           # 4 cm du lidar à l'arrière
SAFETY_BUFFER = 0.15           # 15 cm de marge d'arrêt supplémentaire

# --- Tuning Parameters ---
MAX_LIDAR_DIST = 3.0           # On ignore ce qui est au delà de 3m
# Le rayon de la bulle doit couvrir la partie la plus longue (le devant) + la largeur
ROBOT_BUBBLE_RADIUS = 0.40     # Rayon de sécurité (doit être > LIDAR_TO_FRONT + largeur/2)
LIDAR_ROTATION_OFFSET = 90.0   # Correction de rotation du lidar

# --- Maneuver Logic ---
# Distance déclenchant l'arrêt d'urgence : Longueur nez + Marge
CRITICAL_DISTANCE = LIDAR_TO_FRONT + SAFETY_BUFFER  # ex: 0.16 + 0.15 = 0.31m
STOP_WAIT_TIME = 1.0           # Temps d'arrêt OBLIGATOIRE (protection VESC)
REVERSE_DURATION = 1.5         # Temps de marche arrière
REVERSE_SPEED = -0.15          # Vitesse de recul

class NavState(Enum):
    FORWARD = auto()
    BRAKING_TO_REVERSE = auto()
    REVERSING = auto()
    BRAKING_TO_FORWARD = auto()

class LidarNavigator:
    """
    Navigation controller using Follow the Gap algorithm with Maneuver State Machine.
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
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.safety_distance = safety_distance
        self.field_of_view = field_of_view
        self.lookahead_distance = lookahead_distance
        self.gap_threshold = gap_threshold
        
        # State Machine variables
        self.state = NavState.FORWARD
        self.state_timer = 0.0
        
        # Smoothing and Memory
        self.last_steering = 0.0
        self.alpha_steering = 0.6
        self.last_gap_center_steering = 0.0

    def compute_commands(self, lidar_scan: List[Dict]) -> Tuple[float, float]:
        """
        Main loop required by ManualDriveApp.
        Returns (acceleration, steering).
        """
        current_time = time.monotonic()
        
        # 1. Preprocess Lidar Data
        # Returns array [[angle, dist], ...] sorted by angle
        ranges = self._preprocess_scan(lidar_scan)
        
        if len(ranges) == 0:
            return 0.0, 0.0

        # Calculate metrics
        closest_idx = np.argmin(ranges[:, 1])
        min_dist = ranges[closest_idx, 1]
        
        acceleration = 0.0
        steering = self.last_steering

        # --- STATE MACHINE LOGIC ---

        # STATE 1: FORWARD DRIVING
        if self.state == NavState.FORWARD:
            # Check for blockage using the physical dimensions
            # If closest object is closer than (Nez + Marge), STOP.
            if min_dist < CRITICAL_DISTANCE:
                print(f"[Nav] Obstacle at {min_dist:.2f}m (Front is {LIDAR_TO_FRONT}m). Emergency Stop.")
                self.state = NavState.BRAKING_TO_REVERSE
                self.state_timer = current_time
                acceleration = 0.0
            else:
                # Normal Driving
                acceleration, steering = self._logic_follow_the_gap(ranges, closest_idx, min_dist)
                
                # Apply smoothing
                steering = (self.alpha_steering * steering) + \
                           ((1.0 - self.alpha_steering) * self.last_steering)
                
                # Save steering intention
                self.last_gap_center_steering = steering

        # STATE 2: BRAKING BEFORE REVERSE
        elif self.state == NavState.BRAKING_TO_REVERSE:
            acceleration = 0.0
            steering = self.last_steering
            
            # Mandatory Wait
            if (current_time - self.state_timer) > STOP_WAIT_TIME:
                print("[Nav] Engaging Reverse...")
                self.state = NavState.REVERSING
                self.state_timer = current_time

        # STATE 3: REVERSING
        elif self.state == NavState.REVERSING:
            acceleration = REVERSE_SPEED
            
            # Smart Reverse: Invert steering to un-stuck the nose
            steering = -1.0 * np.sign(self.last_gap_center_steering)
            if abs(steering) < 0.1: steering = 0.0
            
            # Exit condition (Time based)
            if (current_time - self.state_timer) > REVERSE_DURATION:
                self.state = NavState.BRAKING_TO_FORWARD
                self.state_timer = current_time

        # STATE 4: BRAKING BEFORE FORWARD
        elif self.state == NavState.BRAKING_TO_FORWARD:
            acceleration = 0.0
            steering = self.last_steering
            
            # Mandatory Wait
            if (current_time - self.state_timer) > STOP_WAIT_TIME:
                print("[Nav] Engaging Forward...")
                self.state = NavState.FORWARD
                self.last_steering = 0.0 # Reset smoothing

        self.last_steering = steering
        return acceleration, steering

    # --------------------------------------------------------------------------
    # CORE ALGORITHM
    # --------------------------------------------------------------------------

    def _logic_follow_the_gap(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> Tuple[float, float]:
        """Calculates speed and steering for normal forward driving."""
        
        # 1. Apply Safety Bubble
        proc_ranges = self._apply_safety_bubble(ranges, closest_idx, min_dist)
        
        # 2. Find Max Gap (Width based)
        gap_start, gap_end = self._find_max_gap(proc_ranges)
        
        # 3. Find Best Point (Center of gap)
        target_angle = self._find_best_point(proc_ranges, gap_start, gap_end)
        
        # 4. Compute controls
        steering_raw = self._angle_to_steering(target_angle)
        speed = self._compute_speed(min_dist, abs(steering_raw))
        
        return speed, steering_raw

    # --------------------------------------------------------------------------
    # HELPERS
    # --------------------------------------------------------------------------

    def _preprocess_scan(self, scan: List[Dict]) -> np.ndarray:
        data = []
        for p in scan:
            angle = p['angle']
            dist = p['distance']
            
            # Apply rotation correction
            angle = angle + LIDAR_ROTATION_OFFSET
            
            # Normalize angle -180 to 180
            if angle > 180: angle -= 360
            if angle <= -180: angle += 360
            
            if self.field_of_view[0] <= angle <= self.field_of_view[1]:
                # CLAMP
                if dist > MAX_LIDAR_DIST:
                    dist = MAX_LIDAR_DIST
                # Filter noise
                if dist < 0.1:
                    dist = 0.0
                
                data.append([angle, dist])
        
        if not data:
            return np.array([])
            
        arr = np.array(data)
        return arr[arr[:, 0].argsort()]

    def _apply_safety_bubble(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> np.ndarray:
        proc_ranges = ranges.copy()
        
        if min_dist < MAX_LIDAR_DIST:
            if min_dist < 0.1: min_dist = 0.1
            # We assume ROBOT_BUBBLE_RADIUS covers the physical size including offset
            angle_radius_rad = np.arctan(ROBOT_BUBBLE_RADIUS / min_dist)
            angle_radius_deg = np.degrees(angle_radius_rad)
            
            closest_angle = ranges[closest_idx, 0]
            
            mask = np.abs(proc_ranges[:, 0] - closest_angle) < angle_radius_deg
            proc_ranges[mask, 1] = 0.0
            
        return proc_ranges

    def _find_max_gap(self, ranges: np.ndarray) -> Tuple[int, int]:
        mask = ranges[:, 1] > self.gap_threshold
        padded = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0] - 1
        
        if len(starts) == 0:
            return 0, len(ranges) - 1
            
        lengths = ends - starts
        longest_idx = np.argmax(lengths)
        return starts[longest_idx], ends[longest_idx]

    def _find_best_point(self, ranges: np.ndarray, start: int, end: int) -> float:
        gap_points = ranges[start:end+1]
        if len(gap_points) == 0:
            return 0.0
        return gap_points[len(gap_points)//2, 0]

    def _angle_to_steering(self, angle: float) -> float:
        steer = angle / 90.0
        return max(-1.0, min(1.0, steer))

    def _compute_speed(self, closest_dist: float, steer_effort: float) -> float:
        # Stop safely before hitting Critical Distance
        if closest_dist < CRITICAL_DISTANCE:
            return 0.0
            
        # Scale speed based on distance relative to Critical Distance
        # 0.0 speed at Critical Distance, Max speed at Safety Distance
        dist_factor = np.clip((closest_dist - CRITICAL_DISTANCE) / (self.safety_distance), 0.0, 1.0)
        
        turn_factor = 1.0 - (0.6 * steer_effort)
        
        target_speed = self.max_speed * dist_factor * turn_factor
        return max(self.min_speed, target_speed)
