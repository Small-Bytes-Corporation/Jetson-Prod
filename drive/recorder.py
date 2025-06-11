#!/usr/bin/env python3
"""
@file recorder.py
@brief Record dataset samples (rays + joystick input) using DepthAI camera and a trained mask model.
Run with: python3 recorder.py <path_to_mask_model.pth>
"""

import sys
import os
import time
import csv
import signal
from math import sqrt

import cv2
import depthai as dai
import numpy as np
import torch
import torch.nn as nn

# -- Import joystick control from local module --
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))
from init_controller import init_joystick, read_inputs, Input

# -- Import U-Net from models --
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from models.unet import SimpleUNet

# === Constants ===
CAM_WIDTH, CAM_HEIGHT = 320, 180
NUM_RAYS = 24
FOV = 180
MAX_DISTANCE = sqrt(CAM_WIDTH**2 + CAM_HEIGHT**2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]

DELTA_ACC = 0.003
DELTA_BRAKE = 0.006
MAX_SPEED = 0.15

# === Control ===
keep_running = True
def signal_handler(sig, frame):
    global keep_running
    print("\nSignal received. Exiting...")
    keep_running = False
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def compute_target(input_state, max_speed=MAX_SPEED):
    throttle = (input_state[Input.RT] + 1) / 6
    brake = (input_state[Input.LT] + 1) / 7
    clamped_throttle = min(throttle, max_speed)
    clamped_brake = min(brake, -max_speed)
    return max(-max_speed, min(max_speed, clamped_throttle - clamped_brake))

# === Raycasting ===
def perform_raycasting(mask_np: np.ndarray) -> np.ndarray:
    height, width = mask_np.shape
    cx, cy = width // 2, height - 1
    distances = np.full(NUM_RAYS, MAX_DISTANCE, dtype=np.float32)
    half_fov = FOV / 2
    angle_step = FOV / (NUM_RAYS - 1)
    for i in range(NUM_RAYS):
        angle = -half_fov + i * angle_step - 90
        dx, dy = np.cos(np.radians(angle)), np.sin(np.radians(angle))
        x, y = float(cx), float(cy)
        for _ in range(int(MAX_DISTANCE)):
            x += dx; y += dy
            ix, iy = int(x), int(y)
            if not (0 <= ix < width and 0 <= iy < height): break
            if mask_np[iy, ix] > 0:
                distances[i] = sqrt((ix - cx)**2 + (iy - cy)**2)
                break
    return distances

# === Main ===
def main():
    if len(sys.argv) != 2 or sys.argv[1] in {"-h", "--help"}:
        print("Usage: python3 recorder.py <mask_model.pth>")
        sys.exit(1)

    model_path = sys.argv[1]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Load segmentation model
    model = SimpleUNet().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    def preprocess(frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
        img = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1)
        for i in range(3):
            img[i] = (img[i] - IMG_NORM_MEAN[i]) / IMG_NORM_STD[i]
        return img.unsqueeze(0).to(device)

    # DepthAI pipeline setup
    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(CAM_WIDTH, CAM_HEIGHT)
    cam.setInterleaved(False); cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    # Initialize joystick
    js = init_joystick()
    input_state = [0 for _ in range(19)]
    input_state[Input.LT] = -1
    input_state[Input.RT] = -1
    print("Joystick ready:", js.get_name())
    print("[Start] Hold RB to record. Press BACK+START to exit.")

    # Start camera + logging
    with dai.Device(pipeline) as device_cam, open("recorded_drive.csv", "w", newline="") as f:
        writer = csv.writer(f)
        q_rgb = device_cam.getOutputQueue("rgb", 4, blocking=False)
        acceleration = 0.0

        while keep_running:
            frame_in = q_rgb.tryGet()
            if not frame_in:
                time.sleep(0.01)
                continue

            read_inputs(js, input_state)
            if input_state[Input.BACK] and input_state[Input.START]:
                print("Exit requested.")
                break

            frame = frame_in.getCvFrame()
            with torch.no_grad():
                tensor = preprocess(frame)
                mask = torch.sigmoid(model(tensor)).squeeze().cpu().numpy()
                binary_mask = (mask > 0.5).astype(np.uint8) * 255
                rays = perform_raycasting(binary_mask)
                norm_rays = (rays / MAX_DISTANCE).tolist()

            target = compute_target(input_state)
            if acceleration < target:
                acceleration = min(acceleration + DELTA_ACC, target)
            elif acceleration > target:
                acceleration = max(acceleration - DELTA_BRAKE, target)

            steer = input_state[Input.LEFT_JOY_X]
            print("Duty: {:.3f} | Steer: {:.3f} | RT: {:.2f} | LT: {:.2f}".format(
                acceleration, (steer + 1)/2, input_state[Input.RT], input_state[Input.LT]))

            if input_state[Input.RB]:
                row = [acceleration, steer] + norm_rays
                writer.writerow(row)
                print("[REC] Sample saved")

    print("Shutdown complete.")

if __name__ == "__main__":
    main()
