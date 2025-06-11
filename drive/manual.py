# drive/controller.py
"""
Control a VESC motor using a gamepad via pygame.
Usage: python3 controller.py 0.15
"""

import sys
import time
from pyvesc import VESC
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))
from init_controller import init_joystick, read_inputs, Input
from enum import IntEnum


MAX_SPEED = 0.15
DELTA_ACC = 0.003
DELTA_BRAKE = 0.006

class Axis(IntEnum):
    LEFT_JOY_X = 0
    LT = 2
    RT = 5

def apply_motor_commands(motor, acceleration, steer_input):
    steer_value = (steer_input + 1) / 2
    motor.set_servo(steer_value)
    motor.set_duty_cycle(acceleration)

def compute_target(input_state, max_fwd, max_bwd):
    throttle = (input_state[Input.RT] + 1) / 6
    brake = (input_state[Input.LT] + 1) / 7
    clamped_throttle = min(throttle, max_fwd)
    clamped_brake = min(brake, -max_bwd)
    return max(max_bwd, min(max_fwd, clamped_throttle - clamped_brake))

def init_motor(serial_port):
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

def control_loop(max_speed=0.05, serial_port='/dev/ttyACM0'):
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
                acceleration = min(acceleration + DELTA_ACC, target)
            elif acceleration > target:
                acceleration = max(acceleration - DELTA_BRAKE, target)

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
