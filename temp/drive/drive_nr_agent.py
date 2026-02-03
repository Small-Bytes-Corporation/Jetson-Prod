# drive/driver_nragent.py
"""
Autonomous driving using U-Net + NR agent + threading.
Threaded architecture:
- Vision thread performs camera + segmentation + raycasting + action.
- Main loop applies motor commands periodically.
- Press 'A' to immediately stop the vehicle.

Usage:
    python3 drive/driver_nragent.py ai_mask/mask.pth ai_drive/nr_model.pth
"""

import os
import sys
import time
import signal
import threading
import numpy as np
import torch
import cv2
import depthai as dai
from math import sqrt
from pyvesc import VESC

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
MAX_DISTANCE = sqrt(CAM_WIDTH**2 + CAM_HEIGHT**2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]
MAX_SPEED = 0.15
STOP_ACCEL = 0.0
STOP_STEER = 0.0

keep_running = True

signal.signal(signal.SIGINT, lambda s, f: exit(0))
signal.signal(signal.SIGTERM, lambda s, f: exit(0))

def preprocess(frame, device):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
    tensor = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1)
    for i in range(3):
        tensor[i] = (tensor[i] - IMG_NORM_MEAN[i]) / IMG_NORM_STD[i]
    return tensor.unsqueeze(0).to(device)

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
    return distances

def vision_loop(shared, lock, q_rgb, mask_model, agent, device):
    while keep_running:
        frame_in = q_rgb.tryGet()
        if frame_in is None:
            time.sleep(0.01)
            continue
        frame = frame_in.getCvFrame()
        with torch.no_grad():
            input_tensor = preprocess(frame, device)
            mask = torch.sigmoid(mask_model(input_tensor)).squeeze().cpu().numpy()
            binary = (mask > 0.5).astype(np.uint8)
            rays = perform_raycasting(binary)
            normalized = [r / MAX_DISTANCE for r in rays[:NUM_RAYS]]
            acc, steer = agent.select_action(normalized)
        with lock:
            shared['acc'] = float(max(-MAX_SPEED, min(MAX_SPEED, acc)))
            shared['steer'] = float(max(-1.0, min(1.0, steer)))

def init_motor(port='/dev/ttyACM0'):
    for i in range(5):
        try:
            return VESC(serial_port=port)
        except Exception as e:
            print(f"[VESC] Attempt {i+1} failed: {e}")
            time.sleep(1)
    raise RuntimeError("[VESC] Failed to initialize VESC")

def apply_motor_commands(motor, acc, steer):
    motor.set_servo((steer + 1) / 2)
    motor.set_duty_cycle(acc)

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 drive/driver_nragent.py ai_mask/mask.pth ai_drive/nr_model.pth")
        sys.exit(1)

    mask_path, agent_path = sys.argv[1:3]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Load models
    mask_model = SimpleUNet().to(device)
    mask_model.load_state_dict(torch.load(mask_path, map_location=device))
    mask_model.eval()

    agent = NRAgent()
    agent.load_model(agent_path)

    # Setup camera pipeline
    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(CAM_WIDTH, CAM_HEIGHT)
    cam.setInterleaved(False); cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    shared = {'acc': 0.0, 'steer': 0.0}
    lock = threading.Lock()

    js = init_joystick()
    motor = init_motor()
    print("[INFO] Autonomous driving ready. Press A to stop.")

    with dai.Device(pipeline) as cam_dev:
        q_rgb = cam_dev.getOutputQueue("rgb", maxSize=4, blocking=False)
        thread = threading.Thread(target=vision_loop, args=(shared, lock, q_rgb, mask_model, agent, device))
        thread.start()

        while keep_running:
            pygame.event.pump()
            if js.get_button(0):  # Button A
                print("[FAILSAFE] Button A pressed. Stopping car.")
                apply_motor_commands(motor, STOP_ACCEL, STOP_STEER)
                break
            with lock:
                acc = shared['acc']
                steer = shared['steer']
            apply_motor_commands(motor, acc, steer)
            print(f"[DRIVE] acc={acc:+.3f} | steer={(steer+1)/2:.3f}")
            time.sleep(0.03)

    motor.set_rpm(0)
    motor.stop_heartbeat()
    print("[SHUTDOWN] Motor stopped. Goodbye.")

if __name__ == "__main__":
    main()
