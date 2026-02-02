"""
Throttle controller for smooth acceleration and braking.
"""

from .config import MAX_SPEED, DELTA_ACC, DELTA_BRAKE


class ThrottleController:
    """
    Controller for throttle management with smooth interpolation.
    """
    
    def __init__(self, max_speed=MAX_SPEED, delta_acc=DELTA_ACC, delta_brake=DELTA_BRAKE):
        """
        Initialize the throttle controller.
        
        Args:
            max_speed: Maximum forward/backward speed.
            delta_acc: Acceleration increment per update.
            delta_brake: Braking decrement per update.
        """
        self.max_speed = max_speed
        self.max_fwd = max_speed
        self.max_bwd = -max_speed
        self.delta_acc = delta_acc
        self.delta_brake = delta_brake
        self.acceleration = 0.0
    
    def compute_target(self, rt_value, lt_value):
        """
        Compute target throttle based on trigger input values.
        
        Args:
            rt_value: Right trigger value (-1 to 1).
            lt_value: Left trigger value (-1 to 1).
            
        Returns:
            float: Desired throttle value.
        """
        # Convert triggers from [-1, 1] to throttle/brake values
        throttle = (rt_value + 1) / 6  # RT: [-1, 1] -> [0, 1/3]
        brake = (lt_value + 1) / 7     # LT: [-1, 1] -> [0, 2/7]
        
        # Clamp values
        clamped_throttle = min(throttle, self.max_fwd)
        clamped_brake = min(brake, -self.max_bwd)
        
        # Compute target: throttle - brake, clamped to [max_bwd, max_fwd]
        target = max(self.max_bwd, min(self.max_fwd, clamped_throttle - clamped_brake))
        return target
    
    def update(self, target):
        """
        Update acceleration with smooth interpolation towards target.
        
        Args:
            target: Target acceleration value.
        """
        if self.acceleration < target:
            # Accelerating
            self.acceleration = min(self.acceleration + self.delta_acc, target)
        elif self.acceleration > target:
            # Braking
            self.acceleration = max(self.acceleration - self.delta_brake, target)
    
    def get_acceleration(self):
        """
        Get the current acceleration value.
        
        Returns:
            float: Current acceleration.
        """
        return self.acceleration
    
    def reset(self):
        """Reset acceleration to zero."""
        self.acceleration = 0.0
