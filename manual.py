import time
import sys
import pygame
import os
import cv2
from enum import IntEnum
from pyvesc import VESC

serial_port = '/dev/ttyACM0'
CAPTURE_INTERVAL = 0.5
CAPTURE_DIR = "captures"

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
    RIGHT_JOY_X = 3
    RIGHT_JOY_Y = 4
    LT = 2
    RT = 5

MAX_FWD = 0.3
MAX_BWD = -0.15
DEADZONE = 0.05
DELTA_ACC = 0.005
DELTA_BRAKE = 0.008

def vroum():
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    input = [0 for _ in range(19)]
    input[Input.LT] = -1
    input[Input.RT] = -1
    acceleration = 0
    recording = False
    last_capture_time = 0
    capture_index = 0
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("impossible d'ouvrir la caméra.")
        return
    print("\tBouton X pour activer/désactiver l'enregistrement")
    with VESC(serial_port=serial_port) as motor:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                    for btn in range(11):
                        input[btn] = joysticks[0].get_button(btn)
                if event.type == pygame.JOYAXISMOTION:
                    input[Input.LEFT_JOY_X] = round(joysticks[0].get_axis(Axis.LEFT_JOY_X), ndigits=2)
                    input[Input.LEFT_JOY_Y] = round(joysticks[0].get_axis(Axis.LEFT_JOY_Y), ndigits=2)
                    input[Input.RIGHT_JOY_X] = round(joysticks[0].get_axis(Axis.RIGHT_JOY_X), ndigits=2)
                    input[Input.RIGHT_JOY_Y] = round(joysticks[0].get_axis(Axis.RIGHT_JOY_Y), ndigits=2)
                    input[Input.LT] = round(joysticks[0].get_axis(Axis.LT), ndigits=2)
                    input[Input.RT] = round(joysticks[0].get_axis(Axis.RT), ndigits=2)
                if event.type == pygame.JOYHATMOTION:
                    x, y = joysticks[0].get_hat(0)
                    input[Input.HAT_X] = x
                    input[Input.HAT_Y] = y
            if input[Input.X]:
                recording = not recording
                print(f"Recording {'ON' if recording else 'OFF'}")
                time.sleep(0.3)
            value = (input[Input.RT] + 1) / 6
            cl = min(value, MAX_FWD)
            value2 = (input[Input.LT] + 1) / 7
            ca = min(value2, MAX_BWD * -1)


            target = cl - ca
            target = min(MAX_FWD, max(MAX_BWD, target))
            if acceleration < target:
                acceleration = min(acceleration + DELTA_ACC, target)
            elif acceleration > target:
                acceleration = max(acceleration - DELTA_ACC, target)

            if target > 0:
                force = target / MAX_FWD
                joysticks[0].rumble(force + 0.3, force + 0.3, 10)
            elif target < 0:
                force = -target / MAX_FWD
                joysticks[0].rumble(force + 0.45, force + 0.45, 10)

            motor.set_servo((input[Input.LEFT_JOY_X] + 1) / 2)
            motor.set_duty_cycle(target)

            if recording:
                now = time.time()
                if now - last_capture_time >= CAPTURE_INTERVAL:
                    ret, frame = cap.read()
                    if ret:
                        filename = os.path.join(CAPTURE_DIR, f"capture_{capture_index:05d}.jpg")
                        cv2.imwrite(filename, frame)
                        print(f"\tImage sauvegardée : {filename}")
                        capture_index += 1
                        last_capture_time = now
            if input[Input.A]:
                print("leave")
                break
            time.sleep(0.1)

        motor.set_rpm(0)
        motor.stop_heartbeat()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    vroum()
