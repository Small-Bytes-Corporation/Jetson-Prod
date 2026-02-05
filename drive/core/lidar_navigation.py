"""
Advanced Navigation Algorithm - State Machine based Follow the Gap.

Features:
- Reactive 'Follow the Gap' for forward driving.
- Safety Bubble inflation for obstacle avoidance.
- Finite State Machine (FSM) for maneuvers.
- Smart Reverse: Detects dead-ends/blocked paths and performs 3-point turns.
- VESC Protection: Enforces a full stop delay before changing gear (Forward <-> Reverse).
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

# --- Tuning Parameters ---
MAX_LIDAR_DIST = 3.0           # Capteur borné à 3m pour uniformiser les "trous"
ROBOT_BUBBLE_RADIUS = 0.20     # Rayon de sécurité (largeur robot + marge)
CRITICAL_DISTANCE = 0.10       # Distance déclenchant l'arrêt d'urgence et la marche arrière
STOP_WAIT_TIME = 1.0           # Temps d'arrêt OBLIGATOIRE avant changement de sens (protection moteur)
REVERSE_DURATION = 1.5         # Durée de la marche arrière
REVERSE_SPEED = -0.10          # Vitesse de recul (douce)
LIDAR_ROTATION_OFFSET = 90.0  # degrees: lidar is rotated 90° to the right, so add 90° to correct

class NavState(Enum):
    FORWARD = auto()
    BRAKING_TO_REVERSE = auto()
    REVERSING = auto()
    BRAKING_TO_FORWARD = auto()


class LidarNavigator:
    """
    Advanced navigation controller with maneuver capabilities.
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
        
        # State Machine
        self.state = NavState.FORWARD
        self.state_timer = 0.0
        
        # Memory for smoothing and maneuvers
        self.last_steering = 0.0
        self.alpha_steering = 0.6  # 0.0 = pas de changement, 1.0 = pas de lissage
        self.last_gap_center = 0.0 # Pour se souvenir de la direction générale

    def compute_commands(self, lidar_scan: List[Dict]) -> Tuple[float, float]:
        """
        Main loop required by ManualDriveApp.
        Returns (acceleration, steering).
        """
        current_time = time.monotonic()
        
        # 1. Preprocess Lidar Data (Common to all states)
        ranges = self._preprocess_scan(lidar_scan)
        
        if len(ranges) == 0:
            # No data: Panic stop
            return 0.0, 0.0

        # Calculate useful metrics
        closest_idx = np.argmin(ranges[:, 1])
        min_dist = ranges[closest_idx, 1]
        
        # --- STATE MACHINE LOGIC ---
        
        acceleration = 0.0
        steering = self.last_steering

        # STATE 1: FORWARD DRIVING
        if self.state == NavState.FORWARD:
            # Check if blocked
            is_blocked = min_dist < CRITICAL_DISTANCE
            
            if is_blocked:
                # TRANSITION -> BRAKING
                print(f"[Nav] Blocked ({min_dist:.2f}m)! Stopping.")
                self.state = NavState.BRAKING_TO_REVERSE
                self.state_timer = current_time
                acceleration = 0.0
            else:
                # Normal Driving Logic (Follow the Gap)
                acc, steer = self._logic_follow_the_gap(ranges, closest_idx, min_dist)
                acceleration = acc
                steering = steer
                
                # Save context for potential reversing later
                # If we were turning left, we probably want to reverse with wheels right
                self.last_gap_center = steering 

        # STATE 2: BRAKING (Before Reverse)
        elif self.state == NavState.BRAKING_TO_REVERSE:
            acceleration = 0.0 # Force Stop
            steering = self.last_steering # Keep wheels steady
            
            # Wait for motor to fully stop
            if (current_time - self.state_timer) > STOP_WAIT_TIME:
                # TRANSITION -> REVERSE
                print("[Nav] Engaging Reverse Gear.")
                self.state = NavState.REVERSING
                self.state_timer = current_time

        # STATE 3: REVERSING
        elif self.state == NavState.REVERSING:
            acceleration = REVERSE_SPEED
            
            # Smart Steering during reverse:
            # If we were trying to go Left (positive steer), obstacle is likely straight or right.
            # To unstick the nose, we usually want to reverse continuously turning the SAME way 
            # OR opposite way depending on geometry. 
            # Strategy: Invert steering to "back out" of the turn.
            steering = -1.0 * np.sign(self.last_gap_center)
            if abs(steering) < 0.1: steering = 0.0 # If straight, go straight back
            
            # Exit condition: Time elapsed OR plenty of space in front
            # Note: Checking "front space" while reversing is tricky because Lidar is at the front.
            # We mostly rely on time here.
            if (current_time - self.state_timer) > REVERSE_DURATION:
                # TRANSITION -> BRAKING (Before Forward)
                self.state = NavState.BRAKING_TO_FORWARD
                self.state_timer = current_time

        # STATE 4: BRAKING (Before Forward)
        elif self.state == NavState.BRAKING_TO_FORWARD:
            acceleration = 0.0
            steering = self.last_steering
            
            if (current_time - self.state_timer) > STOP_WAIT_TIME:
                print("[Nav] Engaging Forward Gear.")
                self.state = NavState.FORWARD
                # Reset steering smoothing to be reactive immediately
                self.last_steering = 0.0 

        # --- OUTPUT SMOOTHING ---
        # Apply smoothing only to steering, throttle is handled by state machine
        if self.state == NavState.FORWARD:
             self.last_steering = (self.alpha_steering * steering) + \
                                  ((1.0 - self.alpha_steering) * self.last_steering)
        else:
            self.last_steering = steering

        return acceleration, self.last_steering

    # --------------------------------------------------------------------------
    # ALGORITHMIC CORE (Follow The Gap)
    # --------------------------------------------------------------------------
    
    def _logic_follow_the_gap(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> Tuple[float, float]:
        """
        Execute the improved Follow The Gap algorithm.
        Returns (target_speed, target_steering)
        """
        # 1. Apply Safety Bubble (inflate obstacles)
        proc_ranges = self._apply_safety_bubble(ranges, closest_idx, min_dist)
        
        # 2. Find Max Gap (Widest passability)
        gap_start, gap_end = self._find_max_gap(proc_ranges)
        
        # 3. Find Best Point (Center of gap)
        target_angle = self._find_best_point(proc_ranges, gap_start, gap_end)
        
        # 4. Compute controls
        steering = self._angle_to_steering(target_angle)
        speed = self._compute_forward_speed(min_dist, abs(steering))
        
        return speed, steering

    def _preprocess_scan(self, scan: List[Dict]) -> np.ndarray:
        """Filter, sort, clamp and normalize scan data."""
        data = []
        for p in scan:
            angle = p['angle']
            dist = p['distance']
            
            # Apply rotation correction (lidar is rotated 90° to the right)
            angle = angle + LIDAR_ROTATION_OFFSET
            
            # Normalize angle -180 to 180
            if angle > 180: angle -= 360
            if angle <= -180: angle += 360
            
            # FOV Check
            if self.field_of_view[0] <= angle <= self.field_of_view[1]:
                # CLAMP: Max distance (Virtual Horizon)
                if dist > MAX_LIDAR_DIST: dist = MAX_LIDAR_DIST
                # Filter noise
                if dist < 0.05: dist = 0.0
                
                data.append([angle, dist])
        
        if not data: return np.array([])
        
        # Sort by angle
        arr = np.array(data)
        return arr[arr[:, 0].argsort()]

    def _apply_safety_bubble(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> np.ndarray:
        """Zero out points around the closest obstacle based on robot radius."""
        proc_ranges = ranges.copy()
        if min_dist < MAX_LIDAR_DIST:
            if min_dist < 0.1: min_dist = 0.1
            # Angle to cover robot radius
            radius_angle = np.degrees(np.arctan(ROBOT_BUBBLE_RADIUS / min_dist))
            
            closest_angle = ranges[closest_idx, 0]
            # Mask points within radius
            mask = np.abs(proc_ranges[:, 0] - closest_angle) < radius_angle
            proc_ranges[mask, 1] = 0.0
        return proc_ranges

    def _find_max_gap(self, ranges: np.ndarray) -> Tuple[int, int]:
        """Find the widest consecutive gap of non-zero points."""
        mask = ranges[:, 1] > self.gap_threshold
        # Find rising/falling edges
        padded = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded.astype(int))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0] - 1
        
        if len(starts) == 0:
            # Panic: No gap found. Return full range (will likely result in stop)
            return 0, len(ranges) - 1
            
        # Select WIDEST gap (most indices), not deepest
        lengths = ends - starts
        longest_idx = np.argmax(lengths)
        return starts[longest_idx], ends[longest_idx]

    def _find_best_point(self, ranges: np.ndarray, start: int, end: int) -> float:
        """Aim for the center of the selected gap."""
        gap_points = ranges[start:end+1]
        if len(gap_points) == 0: return 0.0
        return gap_points[len(gap_points)//2, 0]

    def _angle_to_steering(self, angle: float) -> float:
        # Map -90..90 to -1..1
        steer = angle / 90.0
        return max(-1.0, min(1.0, steer))

    def _compute_forward_speed(self, min_dist: float, steer_effort: float) -> float:
        """Adapt speed based on obstacle proximity and turning sharpness."""
        # 1. Distance Factor
        dist_factor = np.clip((min_dist - CRITICAL_DISTANCE) / (self.safety_distance), 0.0, 1.0)
        
        # 2. Turn Factor (Slow down in corners)
        turn_factor = 1.0 - (0.6 * steer_effort)
        
        target = self.max_speed * dist_factor * turn_factor
        return max(self.min_speed, target)
