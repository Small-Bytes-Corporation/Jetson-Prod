# drive/driver_nragent.py
"""
Autonomous driver using U-Net + NRAgent.
Applies mask -> raycast -> select_action(state) to drive the robot.
"""

import os
import sys
import time
import signal
import threading
import depthai as dai
import numpy as np
import torch
import cv2
from math import sqrt
from pyvesc import VESC

# === Local imports ===
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "NR_Agent")))
from models.unet import SimpleUNet
from drive.NR_Agent.agent import NRAgent
from drive.NR_Agent.settings import NUM_RAYS

# === Constants ===
CAM_WIDTH, CAM_HEIGHT = 320, 180
FOV = 180
MAX_DISTANCE = sqrt(CAM_WIDTH**2 + CAM_HEIGHT**2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]
SERIAL_PORT = '/dev/ttyACM0'
OUTPUT_LIMIT = 0.05

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
            x += dx; y += dy
            ix, iy = int(x), int(y)
            if not (0 <= ix < w and 0 <= iy < h): break
            if mask_np[iy, ix] > 0:
                distances[i] = sqrt((ix - cx)**2 + (iy - cy)**2)
                break
    return distances

def init_motor(port='/dev/ttyACM0'):
    for i in range(5):
        try:
            return VESC(serial_port=port)
        except Exception as e:
            print(f"[VESC] Attempt {i+1} failed: {e}")
            time.sleep(1)
    raise RuntimeError("[VESC] Failed to initialize")

def apply_motor_commands(motor, acc, steer):
    motor.set_servo((steer + 1) / 2)
    motor.set_duty_cycle(acc)

def preprocess(frame, device):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
    img = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1)
    for i in range(3):
        img[i] = (img[i] - IMG_NORM_MEAN[i]) / IMG_NORM_STD[i]
    return img.unsqueeze(0).to(device)

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 drive/driver_nragent.py mask.pth model.pth")
        sys.exit(1)

    mask_path, agent_path = sys.argv[1], sys.argv[2]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Load models
    unet = SimpleUNet().to(device)
    unet.load_state_dict(torch.load(mask_path, map_location=device))
    unet.eval()

    agent = NRAgent().to(device)
    agent.load_model(agent_path)

    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(CAM_WIDTH, CAM_HEIGHT)
    cam.setInterleaved(False); cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    motor = init_motor()
    print("[INFO] Autonomous driving started.")

    with dai.Device(pipeline) as device_cam:
        q_rgb = device_cam.getOutputQueue("rgb", 4, blocking=False)
        while keep_running:
            frame_in = q_rgb.tryGet()
            if not frame_in:
                time.sleep(0.01)
                continue
            frame = frame_in.getCvFrame()
            with torch.no_grad():
                tensor = preprocess(frame, device)
                mask = torch.sigmoid(unet(tensor)).squeeze().cpu().numpy()
                binary = (mask > 0.5).astype(np.uint8)
                rays = perform_raycasting(binary)
                normed = [r / 500.0 for r in rays[:NUM_RAYS]]
                acc, steer = agent.select_action(normed)
                acc = float(np.clip(acc, -OUTPUT_LIMIT, OUTPUT_LIMIT))
                steer = float(np.clip(steer, -1.0, 1.0))

            apply_motor_commands(motor, acc, steer)
            print(f"[DRIVE] Acc={acc:.3f} | Steer={(steer + 1)/2:.3f}")

    motor.set_rpm(0)
    motor.stop_heartbeat()
    print("[EXIT] Autonomous driving stopped.")

if __name__ == "__main__":
    main()
