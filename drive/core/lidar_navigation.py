"""
Navigation algorithm using lidar data - Follow the Gap algorithm.

The Follow the Gap algorithm is a simple and effective reactive navigation method:
1. Preprocess lidar scan data
2. Find the closest obstacle point
3. Create a safety bubble around it
4. Find the largest gap (free space)
5. Drive toward the center of the gap
"""

import math
import numpy as np
from typing import List, Dict, Tuple, Optional
from .config import (
    AUTONOMOUS_MAX_SPEED, AUTONOMOUS_MIN_SPEED, AUTONOMOUS_SAFETY_DISTANCE,
    AUTONOMOUS_FIELD_OF_VIEW, AUTONOMOUS_LOOKAHEAD_DISTANCE, AUTONOMOUS_GAP_THRESHOLD
)


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
        """
        Initialize the lidar navigator.
        
        Args:
            max_speed: Maximum forward speed (0.0 to 1.0)
            min_speed: Minimum forward speed
            safety_distance: Radius of safety bubble around obstacles (meters)
            field_of_view: Tuple (left_limit, right_limit) in degrees relative to front
            lookahead_distance: Preferred distance to look ahead (meters)
            gap_threshold: Minimum gap size to consider (meters)
        """
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.safety_distance = safety_distance
        self.field_of_view = field_of_view
        self.lookahead_distance = lookahead_distance
        self.gap_threshold = gap_threshold
        
    def compute_commands(self, lidar_scan: List[Dict]) -> Tuple[float, float]:
        """
        Compute acceleration and steering commands from lidar scan.
        
        Args:
            lidar_scan: List of dicts with 'angle' (degrees), 'distance' (meters), 'intensity'
            
        Returns:
            Tuple (acceleration, steering) where:
            - acceleration: float between -1.0 (reverse) and 1.0 (forward)
            - steering: float between -1.0 (left) and 1.0 (right)
        """
        if not lidar_scan or len(lidar_scan) == 0:
            # No data, stop
            return 0.0, 0.0
        
        # Step 1: Filter and sort scan data by angle
        filtered_scan = self._filter_scan(lidar_scan)
        
        if not filtered_scan:
            return 0.0, 0.0
        
        # Step 2: Convert to array format (angle, distance)
        scan_array = self._scan_to_array(filtered_scan)
        
        # Step 3: Find closest point
        closest_idx = np.argmin(scan_array[:, 1])
        closest_distance = scan_array[closest_idx, 1]
        
        # Step 4: Create safety bubble (set distances < safety_distance to 0)
        safe_scan = scan_array.copy()
        for i in range(len(safe_scan)):
            if safe_scan[i, 1] < self.safety_distance:
                safe_scan[i, 1] = 0.0
        
        # Step 5: Find largest gap
        gap_start, gap_end = self._find_largest_gap(safe_scan)
        
        if gap_start is None or gap_end is None:
            # No gap found, stop
            return 0.0, 0.0
        
        # Step 6: Find best point in gap (farthest point or center based on lookahead)
        target_angle = self._find_target_angle(safe_scan, gap_start, gap_end)
        
        # Step 7: Compute steering (normalize angle to [-1, 1])
        # Front = 0°, Left = negative, Right = positive
        steering = self._angle_to_steering(target_angle)
        
        # Step 8: Compute speed based on closest obstacle and gap size
        acceleration = self._compute_speed(safe_scan, gap_start, gap_end, closest_distance)
        
        return acceleration, steering
    
    def _filter_scan(self, scan: List[Dict]) -> List[Dict]:
        """Filter scan to field of view and valid distances."""
        filtered = []
        for point in scan:
            angle = point['angle']
            distance = point['distance']
            
            # Convert angle to [-180, 180] relative to front (0°)
            if angle > 180:
                angle = angle - 360
            
            # Filter by field of view
            if self.field_of_view[0] <= angle <= self.field_of_view[1]:
                # Filter valid distances
                if 0.1 < distance < 10.0:  # Valid range
                    filtered.append({
                        'angle': angle,
                        'distance': distance,
                        'intensity': point.get('intensity', 0)
                    })
        
        return filtered
    
    def _scan_to_array(self, scan: List[Dict]) -> np.ndarray:
        """Convert scan list to numpy array sorted by angle."""
        # Sort by angle
        sorted_scan = sorted(scan, key=lambda x: x['angle'])
        
        # Convert to array: [angle, distance]
        array = np.array([[p['angle'], p['distance']] for p in sorted_scan])
        
        return array
    
    def _find_largest_gap(self, scan_array: np.ndarray) -> Tuple[Optional[int], Optional[int]]:
        """
        Find the largest consecutive gap (non-zero distances) in the scan.
        
        Returns:
            Tuple (start_idx, end_idx) of the largest gap, or (None, None) if no gap found.
        """
        gaps = []
        gap_start = None
        
        for i in range(len(scan_array)):
            if scan_array[i, 1] > self.gap_threshold:  # Valid gap point
                if gap_start is None:
                    gap_start = i
            else:  # Obstacle or invalid
                if gap_start is not None:
                    # Gap ends
                    gap_size = scan_array[gap_start:i, 1].sum()  # Sum of distances in gap
                    gaps.append((gap_start, i - 1, gap_size))
                    gap_start = None
        
        # Handle gap that extends to the end
        if gap_start is not None:
            gap_size = scan_array[gap_start:, 1].sum()
            gaps.append((gap_start, len(scan_array) - 1, gap_size))
        
        if not gaps:
            return None, None
        
        # Find largest gap
        largest_gap = max(gaps, key=lambda x: x[2])
        return largest_gap[0], largest_gap[1]
    
    def _find_target_angle(self, scan_array: np.ndarray, gap_start: int, gap_end: int) -> float:
        """
        Find the target angle within the gap.
        Uses lookahead_distance to prefer points at preferred distance.
        """
        gap_points = scan_array[gap_start:gap_end + 1]
        
        if len(gap_points) == 0:
            # Fallback to center of gap
            center_idx = (gap_start + gap_end) // 2
            return scan_array[center_idx, 0]
        
        # Find point closest to lookahead_distance
        distances = gap_points[:, 1]
        angles = gap_points[:, 0]
        
        # Prefer points near lookahead_distance, but also consider angle (prefer center)
        best_idx = 0
        best_score = float('inf')
        
        for i in range(len(gap_points)):
            dist = distances[i]
            angle = abs(angles[i])  # Absolute angle from center
            
            # Score: combination of distance error and angle preference
            distance_error = abs(dist - self.lookahead_distance)
            angle_penalty = angle / 90.0  # Prefer center (0°) over sides
            
            score = distance_error + angle_penalty
            
            if score < best_score:
                best_score = score
                best_idx = i
        
        return angles[best_idx]
    
    def _angle_to_steering(self, angle: float) -> float:
        """
        Convert angle in degrees to steering command [-1, 1].
        -90° (left) -> -1.0, 0° (center) -> 0.0, +90° (right) -> 1.0
        """
        # Normalize angle to [-90, 90]
        angle = max(-90, min(90, angle))
        
        # Convert to [-1, 1]
        steering = angle / 90.0
        return steering
    
    def _compute_speed(self, scan_array: np.ndarray, gap_start: int, gap_end: int, closest_distance: float) -> float:
        """
        Compute acceleration based on closest obstacle and gap size.
        """
        # Base speed on closest distance
        if closest_distance < self.safety_distance:
            return 0.0  # Too close, stop
        
        # Speed factor based on distance (closer = slower)
        distance_factor = min(1.0, (closest_distance - self.safety_distance) / 2.0)
        
        # Gap size factor (larger gap = faster)
        gap_points = scan_array[gap_start:gap_end + 1]
        gap_size = len(gap_points)
        max_gap_size = len(scan_array)
        gap_factor = gap_size / max_gap_size if max_gap_size > 0 else 0.0
        
        # Combine factors
        speed_factor = (distance_factor + gap_factor) / 2.0
        speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor
        
        return min(self.max_speed, max(self.min_speed, speed))
