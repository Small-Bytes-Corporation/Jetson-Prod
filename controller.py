"""
@file controller.py
@brief Control a VESC motor using a gamepad via pygame.

Run with: python3 controller.py 0.15
"""

import sys
import time
import pygame
from enum import IntEnum
from pyvesc import VESC


class Input(IntEnum):
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
    LEFT_JOY_X = 0
    LEFT_JOY_Y = 1
    LT = 2
    RIGHT_JOY_X = 3
    RIGHT_JOY_Y = 4
    RT = 5


def init_joystick():
    """Initializes and returns the first detected joystick."""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected.")
    js = pygame.joystick.Joystick(0)
    js.init()
    return js


def safe_get_axis(js, axis_index, default=0.0):
    """Returns axis value if valid, else returns default."""
    if axis_index < js.get_numaxes():
        return round(js.get_axis(axis_index), 2)
    return default


def read_inputs(js, input_state):
    """Updates input state from pygame events."""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            raise KeyboardInterrupt()
        if event.type in [pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP]:
            for i in range(11):
                input_state[i] = js.get_button(i)
        if event.type == pygame.JOYAXISMOTION:
            input_state[Input.LEFT_JOY_X] = safe_get_axis(js, Axis.LEFT_JOY_X)
            input_state[Input.LEFT_JOY_Y] = safe_get_axis(js, Axis.LEFT_JOY_Y)
            input_state[Input.RIGHT_JOY_X] = safe_get_axis(js, Axis.RIGHT_JOY_X)
            input_state[Input.RIGHT_JOY_Y] = safe_get_axis(js, Axis.RIGHT_JOY_Y)
            input_state[Input.LT] = safe_get_axis(js, Axis.LT, -1)
            input_state[Input.RT] = safe_get_axis(js, Axis.RT, -1)
        if event.type == pygame.JOYHATMOTION:
            hat_x, hat_y = js.get_hat(0)
            input_state[Input.HAT_X] = hat_x
            input_state[Input.HAT_Y] = hat_y


def apply_motor_commands(motor, acceleration, steer_input):
    """Applies computed acceleration and steering to the motor."""
    steer_value = (steer_input + 1) / 2
    motor.set_servo(steer_value)
    motor.set_duty_cycle(acceleration)


def compute_target(input_state, max_fwd, max_bwd):
    """Computes target acceleration from trigger inputs."""
    throttle = (input_state[Input.RT] + 1) / 6
    brake = (input_state[Input.LT] + 1) / 7
    clamped_throttle = min(throttle, max_fwd)
    clamped_brake = min(brake, -max_bwd)
    return max(max_bwd, min(max_fwd, clamped_throttle - clamped_brake))


def init_motor(serial_port):
    """Initializes and returns the VESC motor object."""
    for attempt in range(5):
        try:
            motor = VESC(serial_port=serial_port)
            version = motor.get_firmware_version()
            if version:
                print("Firmware version:", version)
            else:
                print("Warning: Firmware version is None")
            return motor
        except Exception as e:
            print(f"VESC init failed (try {attempt+1}/5):", e)
            time.sleep(1)
    raise RuntimeError("Failed to initialize VESC after 5 attempts.")


def control_loop(max_speed=0.05, delta_acc=0.003, delta_brake=0.006, serial_port='/dev/ttyACM0'):
    """Main control loop for reading joystick and controlling motor."""
    max_fwd = max_speed
    max_bwd = -max_speed
    js = init_joystick()
    print("Joystick initialized:", js.get_name())

    input_state = [0 for _ in range(19)]
    input_state[Input.LT] = -1
    input_state[Input.RT] = -1
    acceleration = 0.0

    motor = init_motor(serial_port)

    try:
        while True:
            read_inputs(js, input_state)
            target = compute_target(input_state, max_fwd, max_bwd)

            if acceleration < target:
                acceleration = min(acceleration + delta_acc, target)
            elif acceleration > target:
                acceleration = max(acceleration - delta_brake, target)

            apply_motor_commands(motor, acceleration, input_state[Input.LEFT_JOY_X])
            print("Duty: {:.3f} | Steer: {:.3f}".format(acceleration, (input_state[Input.LEFT_JOY_X] + 1) / 2))
            time.sleep(0.05)

            if input_state[Input.BACK] and input_state[Input.START]:
                raise KeyboardInterrupt()

    except KeyboardInterrupt:
        print("Stopping motor...")
    finally:
        motor.set_rpm(0)
        motor.stop_heartbeat()
        print("Shutdown complete.")


if __name__ == "__main__":
    try:
        max_speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
    except ValueError:
        print("Usage: python3 controller.py <max_speed:float>")
        sys.exit(1)
    control_loop(max_speed=max_speed)
