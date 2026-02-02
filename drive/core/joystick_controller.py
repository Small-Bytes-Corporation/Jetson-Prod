"""
Joystick controller for gamepad input handling.
"""

import pygame
from enum import IntEnum


class Input(IntEnum):
    """
    Enum representing common controller buttons and axes.
    Indexed from 0 to be used in input state list.
    """
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    LOGI = 8
    LEFT_JOY = 9
    RIGHT_JOY = 10
    HAT_Y = 11
    HAT_X = 12
    LEFT_JOY_X = 13
    LEFT_JOY_Y = 14
    RIGHT_JOY_X = 15
    RIGHT_JOY_Y = 16
    LT = 17
    RT = 18


class Axis(IntEnum):
    """
    Enum for joystick analog axis indices (pygame-specific layout).
    """
    LEFT_JOY_X = 0
    LEFT_JOY_Y = 1
    LT = 2
    RIGHT_JOY_X = 3
    RIGHT_JOY_Y = 4
    RT = 5


class JoystickController:
    """
    Controller for joystick/gamepad input using pygame.
    """
    
    def __init__(self):
        """Initialize the joystick controller."""
        self.js = None
        self.input_state = [0 for _ in range(19)]
        # Initialize triggers to -1 (not pressed)
        self.input_state[Input.LT] = -1
        self.input_state[Input.RT] = -1
        self._initialized = False
    
    def initialize(self):
        """
        Initialize pygame and the first available joystick.
        
        Raises:
            RuntimeError: If no joystick is detected.
        """
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected.")
        
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self._initialized = True
        print(f"[Joystick] Initialized: {self.js.get_name()}")
    
    def _safe_get_axis(self, axis_index, default=0.0):
        """
        Get a joystick axis value if available, else return a default value.
        
        Args:
            axis_index: Axis index to read.
            default: Value to return if axis is out of range.
            
        Returns:
            float: Axis value rounded to 2 decimals.
        """
        if self.js is None:
            return default
        if axis_index < self.js.get_numaxes():
            return round(self.js.get_axis(axis_index), 2)
        return default
    
    def update(self):
        """
        Update the input state by processing pygame events.
        This should be called in the main loop.
        """
        if not self._initialized:
            return
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt()
            
            # Button state (pressed or released)
            if event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP]:
                for i in range(11):
                    if i < self.js.get_numbuttons():
                        self.input_state[i] = self.js.get_button(i)
            
            # Axis motion (analog values)
            if event.type == pygame.JOYAXISMOTION:
                self.input_state[Input.LEFT_JOY_X] = self._safe_get_axis(Axis.LEFT_JOY_X)
                self.input_state[Input.LEFT_JOY_Y] = self._safe_get_axis(Axis.LEFT_JOY_Y)
                self.input_state[Input.RIGHT_JOY_X] = self._safe_get_axis(Axis.RIGHT_JOY_X)
                self.input_state[Input.RIGHT_JOY_Y] = self._safe_get_axis(Axis.RIGHT_JOY_Y)
                self.input_state[Input.LT] = self._safe_get_axis(Axis.LT, -1)
                self.input_state[Input.RT] = self._safe_get_axis(Axis.RT, -1)
            
            # D-pad / hat switch
            if event.type == pygame.JOYHATMOTION:
                if self.js.get_numhats() > 0:
                    hat_x, hat_y = self.js.get_hat(0)
                    self.input_state[Input.HAT_X] = hat_x
                    self.input_state[Input.HAT_Y] = hat_y
    
    def get_axis(self, axis):
        """
        Get the current value of a joystick axis.
        
        Args:
            axis: Axis enum value (from Input or Axis enum).
            
        Returns:
            float: Current axis value.
        """
        if isinstance(axis, Axis):
            # Map Axis enum to Input enum
            axis_map = {
                Axis.LEFT_JOY_X: Input.LEFT_JOY_X,
                Axis.LEFT_JOY_Y: Input.LEFT_JOY_Y,
                Axis.RIGHT_JOY_X: Input.RIGHT_JOY_X,
                Axis.RIGHT_JOY_Y: Input.RIGHT_JOY_Y,
                Axis.LT: Input.LT,
                Axis.RT: Input.RT,
            }
            axis = axis_map.get(axis, axis)
        
        if isinstance(axis, Input) and axis.value < len(self.input_state):
            return self.input_state[axis.value]
        return 0.0
    
    def get_button(self, button):
        """
        Get the current state of a button.
        
        Args:
            button: Button enum value (from Input enum).
            
        Returns:
            bool: True if button is pressed, False otherwise.
        """
        if isinstance(button, Input) and button.value < len(self.input_state):
            return bool(self.input_state[button.value])
        return False
    
    def get_rt(self):
        """Get right trigger value."""
        return self.get_axis(Input.RT)
    
    def get_lt(self):
        """Get left trigger value."""
        return self.get_axis(Input.LT)
    
    def get_steering(self):
        """Get steering value (left joystick X axis)."""
        return self.get_axis(Input.LEFT_JOY_X)
    
    def get_name(self):
        """Get the name of the joystick."""
        if self.js is not None:
            return self.js.get_name()
        return "Unknown"
