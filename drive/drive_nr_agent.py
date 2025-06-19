import os
import sys
import time
import signal
import threading
import numpy as np
import cv2
import torch
import torch.nn as nn
import depthai as dai
from math import sqrt
from pyvesc import VESC
import pygame

# === Local imports ===
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "NR_Agent")))
from models.unet import SimpleUNet
from drive.NR_Agent.agent import NRAgent
from drive.init_controller import init_joystick, Axis
from drive.NR_Agent.settings import NUM_RAYS

# === Constants ===
CAM_WIDTH, CAM_HEIGHT = 320, 180
FOV = 180
MAX_DISTANCE = sqrt(CAM_WIDTH ** 2 + CAM_HEIGHT ** 2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]
SERIAL_PORT = "/dev/ttyACM0"

keep_running = True
fail_safe = False

# === Signal Handling ===
def signal_handler(sig, frame):
    global keep_running
    print("\n[Signal] Stopping...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# === Raycasting ===
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
            x += dx; y += dy
            ix, iy = int(x), int(y)
            if not (0 <= ix < w and 0 <= iy < h): break
            if mask_np[iy, ix] > 0:
                distances[i] = sqrt((ix - cx) ** 2 + (iy - cy) ** 2)
                break
    return distances

# === VESC Control ===
def init_motor(port=SERIAL_PORT):
    for i in range(5):
        try:
            return VESC(serial_port=port)
        except Exception as e:
            print(f"[VESC] Attempt {i+1} failed: {e}")
            time.sleep(1)
    raise RuntimeError("[VESC] Failed to initialize")

def apply_motor(motor, acc, steer):
    steer_val = (steer + 1) / 2
    motor.set_servo(steer_val)
    motor.set_duty_cycle(acc)

# === Preprocess ===
def preprocess(frame, device):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
    img = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1)
    for i in range(3):
        img[i] = (img[i] - IMG_NORM_MEAN[i]) / IMG_NORM_STD[i]
    return img.unsqueeze(0).to(device)

# === Main ===
def main():
    global fail_safe

    if len(sys.argv) != 3:
        print("Usage: python3 driver_nragent.py <mask_model.pth> <agent_model.pth>")
        sys.exit(1)

    pygame.init()
    joystick = init_joystick()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    mask_model = SimpleUNet().to(device)
    mask_model.load_state_dict(torch.load(sys.argv[1], map_location=device))
    mask_model.eval()

    agent = NRAgent().to(device)
    agent.load_model(sys.argv[2])

    motor = init_motor()
    print("[INFO] VESC ready")

    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(CAM_WIDTH, CAM_HEIGHT)
    cam.setInterleaved(False)
    cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    with dai.Device(pipeline) as device_cam:
        q_rgb = device_cam.getOutputQueue("rgb", 4, blocking=False)
        print("[INFO] Running autonomous driver with fail-safe mode (press A to stop)...")

        while keep_running:
            frame_in = q_rgb.tryGet()
            pygame.event.pump()
            if joystick.get_button(0):  # A button
                fail_safe = True
                print("[FAIL-SAFE] Emergency stop activated!")

            if not frame_in:
                continue

            frame = frame_in.getCvFrame()
            with torch.no_grad():
                input_tensor = preprocess(frame, device)
                mask = torch.sigmoid(mask_model(input_tensor)).squeeze().cpu().numpy()
                binary = (mask > 0.5).astype(np.uint8)
                rays = perform_raycasting(binary)
                state = [float(x) / 500.0 for x in rays[:NUM_RAYS]]
                action = agent.select_action(state)

            acc = 0.0 if fail_safe else float(action[0])
            steer = float(action[1])
            apply_motor(motor, acc, steer)
            print(f"[DRIVE] acc: {acc:.3f}, steer: {steer:.3f}, fail_safe: {fail_safe}")
            time.sleep(0.03)

    motor.set_rpm(0)
    motor.stop_heartbeat()
    print("[CLEANUP] Shutdown complete.")

if __name__ == "__main__":
    main()
