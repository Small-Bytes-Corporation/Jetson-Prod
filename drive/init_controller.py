"""
drive/joystick_input.py

Module to handle gamepad input (compatible with pygame).
Provides consistent Input enum, axis mappings, and update logic.
Usable in both controller.py and recorder.py.
"""

import pygame
from enum import IntEnum

# === Input Button Mapping ===
class Input(IntEnum):
    """
    Enum representing common controller buttons and axes.
    Indexed from 0 to be used in input state list.
    """
    A = 0; B = 1; X = 2; Y = 3
    LB = 4; RB = 5; BACK = 6; START = 7
    LOGI = 8; LEFT_JOY = 9; RIGHT_JOY = 10
    HAT_Y = 11; HAT_X = 12
    LEFT_JOY_X = 13; LEFT_JOY_Y = 14
    RIGHT_JOY_X = 15; RIGHT_JOY_Y = 16
    LT = 17; RT = 18

# === Axis Mapping ===
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

# === Joystick Initialization ===
def init_joystick():
    """
    Initialize the first available joystick using pygame.
    Raises RuntimeError if no joystick is found.
    """
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected.")
    js = pygame.joystick.Joystick(0)
    js.init()
    return js

# === Safe Axis Reading ===
def safe_get_axis(js, axis_index, default=0.0):
    """
    Get a joystick axis value if available, else return a default value.

    Args:
        js: The pygame joystick object.
        axis_index: Axis index to read.
        default: Value to return if axis is out of range.

    Returns:
        float: Axis value rounded to 2 decimals.
    """
    if axis_index < js.get_numaxes():
        return round(js.get_axis(axis_index), 2)
    return default

# === Input State Update ===
def read_inputs(js, input_state):
    """
    Update the input_state list with current joystick inputs.

    Args:
        js: The initialized pygame joystick.
        input_state: A list of values indexed by Input enum.
    """
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            raise KeyboardInterrupt()

        # Button state (pressed or released)
        if event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP]:
            for i in range(11):
                input_state[i] = js.get_button(i)

        # Axis motion (analog values)
        if event.type == pygame.JOYAXISMOTION:
            input_state[Input.LEFT_JOY_X] = safe_get_axis(js, Axis.LEFT_JOY_X)
            input_state[Input.LEFT_JOY_Y] = safe_get_axis(js, Axis.LEFT_JOY_Y)
            input_state[Input.RIGHT_JOY_X] = safe_get_axis(js, Axis.RIGHT_JOY_X)
            input_state[Input.RIGHT_JOY_Y] = safe_get_axis(js, Axis.RIGHT_JOY_Y)
            input_state[Input.LT] = safe_get_axis(js, Axis.LT, -1)
            input_state[Input.RT] = safe_get_axis(js, Axis.RT, -1)

        # D-pad / hat switch
        if event.type == pygame.JOYHATMOTION:
            hat_x, hat_y = js.get_hat(0)
            input_state[Input.HAT_X] = hat_x
            input_state[Input.HAT_Y] = hat_y
