"""
Navigation algorithm using lidar data - Follow the Gap algorithm.
Refactored for robustness:
1. True obstacle inflation (Bubble) to account for robot radius.
2. Gap selection based on width (passability), not depth.
3. Range clamping to prevent chasing distant noise.
"""

import math
import numpy as np
from typing import List, Dict, Tuple, Optional
from .config import (
    AUTONOMOUS_MAX_SPEED, AUTONOMOUS_MIN_SPEED, AUTONOMOUS_SAFETY_DISTANCE,
    AUTONOMOUS_FIELD_OF_VIEW, AUTONOMOUS_LOOKAHEAD_DISTANCE, AUTONOMOUS_GAP_THRESHOLD
)

# Constants for preprocessing
MAX_LIDAR_DIST = 3.0  # Cap distances to 3m to view open space uniformly
ROBOT_BUBBLE_RADIUS = 0.35  # meters (Robot radius + Safety margin)
LIDAR_ROTATION_OFFSET = 90.0  # degrees: lidar is rotated 90° to the right, so add 90° to correct

class LidarNavigator:
    """
    Navigation controller using Follow the Gap algorithm.
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
        
        # Smoothing variables
        self.last_steering = 0.0
        self.alpha_steering = 0.7  # Smoothing factor (0.0 = no new data, 1.0 = no history)

    def compute_commands(self, lidar_scan: List[Dict]) -> Tuple[float, float]:
        if not lidar_scan:
            return 0.0, 0.0
        
        # 1. Preprocess: Filter FOV and limit max range
        # ranges elements: [angle_deg, distance_m]
        ranges = self._preprocess_scan(lidar_scan)
        if ranges.shape[0] == 0:
            return 0.0, 0.0

        # 2. Find nearest obstacle
        closest_idx = np.argmin(ranges[:, 1])
        closest_dist = ranges[closest_idx, 1]
        
        # 3. Create Safety Bubble (Obstacle Inflation)
        # We zero out distances inside the bubble so they are treated as walls
        proc_ranges = self._apply_safety_bubble(ranges, closest_idx, closest_dist)
        
        # 4. Find the max gap (based on WIDTH, not depth)
        gap_start, gap_end = self._find_max_gap(proc_ranges)
        
        # 5. Find target point in the chosen gap
        target_angle = self._find_best_point(proc_ranges, gap_start, gap_end)
        
        # 6. Compute commands
        steering_raw = self._angle_to_steering(target_angle)
        
        # Apply smoothing to steering
        self.last_steering = (self.alpha_steering * steering_raw) + \
                             ((1.0 - self.alpha_steering) * self.last_steering)
        
        speed = self._compute_speed(closest_dist, abs(self.last_steering))
        
        return speed, self.last_steering

    def _preprocess_scan(self, scan: List[Dict]) -> np.ndarray:
        """
        Convert list of dicts to numpy array, filter FOV, and clamp max distances.
        """
        # Extract and sort by angle
        data = []
        for p in scan:
            angle = p['angle']
            dist = p['distance']
            
            # Apply rotation correction (lidar is rotated 90° to the right)
            angle = angle + LIDAR_ROTATION_OFFSET
            
            # Normalize angle -180 to 180
            if angle > 180: angle -= 360
            if angle <= -180: angle += 360
            
            if self.field_of_view[0] <= angle <= self.field_of_view[1]:
                # CLAMP: Treat everything > 3m as 3m. 
                # This makes "open space" look like a uniform wall at 3m.
                if dist > MAX_LIDAR_DIST:
                    dist = MAX_LIDAR_DIST
                # Filter noise
                if dist < 0.1:
                    dist = 0.0
                
                data.append([angle, dist])
        
        if not data:
            return np.array([])
            
        # Sort by angle to ensure indices correspond to physical adjacency
        arr = np.array(data)
        arr = arr[arr[:, 0].argsort()]
        return arr

    def _apply_safety_bubble(self, ranges: np.ndarray, closest_idx: int, min_dist: float) -> np.ndarray:
        """
        Zero out data points within a radius of the closest obstacle 
        to account for robot width.
        """
        proc_ranges = ranges.copy()
        
        # If obstacle is too close, create a bubble
        if min_dist < MAX_LIDAR_DIST:
            # Calculate angle required to cover the radius
            # arc = radius / distance -> angle = arctan(radius/dist)
            if min_dist < 0.1: min_dist = 0.1
            angle_radius_rad = np.arctan(ROBOT_BUBBLE_RADIUS / min_dist)
            angle_radius_deg = np.degrees(angle_radius_rad)
            
            # Identify indices covered by this angle
            closest_angle = ranges[closest_idx, 0]
            
            # Simple thresholding on angle difference
            # (Works because we sorted the array in preprocess)
            mask = np.abs(proc_ranges[:, 0] - closest_angle) < angle_radius_deg
            
            # Set distances to 0 (wall) for points inside the bubble
            proc_ranges[mask, 1] = 0.0
            
        return proc_ranges

    def _find_max_gap(self, ranges: np.ndarray) -> Tuple[int, int]:
        """
        Find the widest consecutive gap of non-zero points.
        Returns start and end indices.
        """
        # Create a boolean mask where dist > threshold
        # We use a threshold slightly above 0 because of bubble zeroing
        mask = ranges[:, 1] > self.gap_threshold
        
        # Find consecutive sequences
        # diff(mask) gives 1 where it starts, -1 where it ends
        # We pad to detect start/end at array edges
        padded = np.concatenate(([False], mask, [False]))
        diff = np.diff(padded.astype(int))
        
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0] - 1
        
        if len(starts) == 0:
            return 0, len(ranges) - 1 # No gap found, return whole range (panic)
            
        # Select gap with largest number of points (widest)
        # Previous code used sum() (deepest), which is dangerous.
        lengths = ends - starts
        longest_idx = np.argmax(lengths)
        
        return starts[longest_idx], ends[longest_idx]

    def _find_best_point(self, ranges: np.ndarray, start: int, end: int) -> float:
        """
        Find the target angle. 
        Strategy: Aim for the furthest point in the selected gap, 
        smoothed towards the center of the gap.
        """
        gap_points = ranges[start:end+1]
        
        if len(gap_points) == 0:
            return 0.0

        # Simple robust strategy: Aim for the center of the largest gap
        # This keeps the robot equidistant from obstacles on left and right
        return gap_points[len(gap_points)//2, 0]

    def _angle_to_steering(self, angle: float) -> float:
        # Map -90..90 to -1..1
        steer = angle / 90.0
        return max(-1.0, min(1.0, steer))

    def _compute_speed(self, closest_dist: float, steer_effort: float) -> float:
        """
        Speed logic:
        1. Slow down if obstacles are close.
        2. Slow down if steering hard (turning).
        """
        if closest_dist < self.safety_distance:
            return 0.0
            
        # Distance factor: 0.0 at safety_dist, 1.0 at 1.0m+safety
        dist_factor = np.clip((closest_dist - self.safety_distance), 0.0, 1.0)
        
        # Turn factor: 1.0 at straight, 0.3 at full turn
        turn_factor = 1.0 - (0.7 * steer_effort)
        
        target_speed = self.max_speed * dist_factor * turn_factor
        
        return max(self.min_speed, target_speed)
