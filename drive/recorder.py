# drive/recorder.py

import os
import sys
import time
import signal
import threading
import pygame
import depthai as dai
import numpy as np
import torch
import cv2
from math import sqrt
from pyvesc import VESC

# Local imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from models.unet import SimpleUNet
from drive.init_controller import init_joystick, Axis
from drive.NR_Agent.data_bufferNR import DataBufferNR
from drive.NR_Agent.settings import NUM_RAYS

# Constants
CAM_WIDTH, CAM_HEIGHT = 320, 180
FOV = 180
MAX_DISTANCE = sqrt(CAM_WIDTH**2 + CAM_HEIGHT**2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]
MAX_SPEED = 0.15
DELTA_ACC = 0.003
DELTA_BRAKE = 0.006

keep_running = True

def signal_handler(sig, frame):
    global keep_running
    print("\n[Signal] Exiting...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def perform_raycasting(mask_np):
    h, w = mask_np.shape
    cx, cy = w // 2, h - 1
    distances = np.full(NUM_RAYS, MAX_DISTANCE, dtype=np.float32)
    angle_step = FOV / (NUM_RAYS - 1)
    for i in range(NUM_RAYS):
        angle = np.deg2rad(-FOV / 2 + i * angle_step - 90)
        dx, dy = np.cos(angle), np.sin(angle)
        x, y = float(cx), float(cy)
        for _ in range(int(MAX_DISTANCE)):
            x += dx
            y += dy
            ix, iy = int(x), int(y)
            if not (0 <= ix < w and 0 <= iy < h): break
            if mask_np[iy, ix] > 0:
                distances[i] = sqrt((ix - cx)**2 + (iy - cy)**2)
                break
    return distances.tolist()

def init_motor(port='/dev/ttyACM0'):
    for i in range(5):
        try:
            motor = VESC(serial_port=port)
            print("Connected to VESC.")
            return motor
        except Exception as e:
            print(f"[VESC] Attempt {i+1}: {e}")
            time.sleep(1)
    raise RuntimeError("Failed to init VESC.")

def apply_motor_commands(motor, acc, steer):
    steer_val = (steer + 1) / 2
    motor.set_servo(steer_val)
    motor.set_duty_cycle(acc)

def vision_loop(shared, lock, device_cam, model, preprocess):
    q_rgb = device_cam.getOutputQueue("rgb", 4, blocking=False)
    while keep_running:
        frame_in = q_rgb.tryGet()
        if not frame_in:
            time.sleep(0.005)
            continue
        frame = frame_in.getCvFrame()
        with torch.no_grad():
            tensor = preprocess(frame)
            mask = torch.sigmoid(model(tensor)).squeeze().cpu().numpy()
            binary_mask = (mask > 0.5).astype(np.uint8)
            rays = perform_raycasting(binary_mask)
        with lock:
            shared['lidar'] = rays

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 drive/recorder.py ai_mask/mask.pth")
        sys.exit(1)

    model_path = sys.argv[1]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SimpleUNet().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    def preprocess(frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
        img = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1)
        for i in range(3):
            img[i] = (img[i] - IMG_NORM_MEAN[i]) / IMG_NORM_STD[i]
        return img.unsqueeze(0).to(device)

    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(CAM_WIDTH, CAM_HEIGHT)
    cam.setInterleaved(False)
    cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    js = init_joystick()
    motor = init_motor()
    print("Joystick + VESC ready. Hold RB to record, BACK+START to quit.")

    shared = {'lidar': [MAX_DISTANCE] * NUM_RAYS}
    lock = threading.Lock()
    acceleration = 0.0
    data_buffer = DataBufferNR()

    with dai.Device(pipeline) as device_cam:
        vision_thread = threading.Thread(target=vision_loop, args=(shared, lock, device_cam, model, preprocess))
        vision_thread.start()

        while keep_running:
            pygame.event.pump()
            steer = js.get_axis(Axis.LEFT_JOY_X)
            rt = js.get_axis(Axis.RT)
            lt = js.get_axis(Axis.LT)
            back = js.get_button(6)
            start = js.get_button(7)
            rb = js.get_button(5)

            if back and start:
                print("Exit requested.")
                break

            throttle = (rt + 1) / 6
            brake = (lt + 1) / 7
            target = max(-MAX_SPEED, min(MAX_SPEED, throttle - brake))

            if acceleration < target:
                acceleration = min(acceleration + DELTA_ACC, target)
            elif acceleration > target:
                acceleration = max(acceleration - DELTA_BRAKE, target)

            apply_motor_commands(motor, acceleration, steer)

            with lock:
                rays = shared['lidar']

            print(f"Duty: {acceleration:.3f} | Steer: {(steer + 1)/2:.3f} | RB: {rb}")

            if rb:
                data_map = {'lidar': rays}
                data_buffer.add_to_buffer(data_map, [acceleration, steer])
                print("[REC] Added sample")

    motor.set_rpm(0)
    motor.stop_heartbeat()
    data_buffer.save_data()
    print("Shutdown complete.")

if __name__ == "__main__":
    main()
