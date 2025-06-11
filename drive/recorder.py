# drive/recorder.py
"""
Multithreaded dataset recorder using joystick input and segmentation model.

- Captures RGB frames from DepthAI camera.
- Runs U-Net mask inference in a separate thread.
- Raycasts from mask to generate distance-like data.
- Logs acceleration, steering, and ray results to CSV.

Usage:
    python3 drive/recorder.py ai_mask/mask.pth
"""

# === Imports ===
import os
import sys
import time
import csv
import signal
import threading
import pygame
import depthai as dai
import numpy as np
import torch
import cv2
from math import sqrt
from pyvesc import VESC

# --- Local imports ---
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from models.unet import SimpleUNet
from drive.init_controller import init_joystick, read_inputs, Input, Axis

# === Constants ===
CAM_WIDTH, CAM_HEIGHT = 320, 180
NUM_RAYS = 24
FOV = 180
MAX_DISTANCE = sqrt(CAM_WIDTH**2 + CAM_HEIGHT**2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]
MAX_SPEED = 0.15
DELTA_ACC = 0.003
DELTA_BRAKE = 0.006

keep_running = True

# === Signal Handling ===
def signal_handler(sig, frame):
    """Handles CTRL+C or termination signals for graceful shutdown."""
    global keep_running
    print("\nSignal received. Exiting...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# === Raycasting Utility ===
def perform_raycasting(mask_np: np.ndarray) -> np.ndarray:
    """
    Casts NUM_RAYS from the bottom center of the mask image, detecting
    the first white pixel hit and returning the normalized distance.
    """
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

# === Motor Control ===
def init_motor(port='/dev/ttyACM0'):
    """Tries to connect to the VESC motor over serial (up to 5 attempts)."""
    for i in range(5):
        try:
            motor = VESC(serial_port=port)
            print("Connected to VESC.")
            return motor
        except Exception as e:
            print(f"Attempt {i+1}: {e}")
            time.sleep(1)
    raise RuntimeError("Failed to init VESC.")

def apply_motor_commands(motor, acc, steer):
    """Sends throttle and steering commands to the motor."""
    steer_val = (steer + 1) / 2  # Normalize [-1,1] to [0,1]
    motor.set_servo(steer_val)
    motor.set_duty_cycle(acc)

# === Vision Thread ===
def vision_loop(shared, lock, device_cam, model, preprocess):
    """
    Separate thread for camera capture + inference.
    Saves normalized ray distances to shared dict (protected by lock).
    """
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
            binary_mask = (mask > 0.5).astype(np.uint8) * 255
            rays = perform_raycasting(binary_mask)
        with lock:
            shared['rays'] = (rays / MAX_DISTANCE).tolist()

# === Main Recorder Logic ===
def main():
    if len(sys.argv) != 2:
        print("Usage: python3 drive/recorder.py ai_mask/mask.pth")
        sys.exit(1)

    # --- Load model ---
    model_path = sys.argv[1]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SimpleUNet().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    def preprocess(frame):
        """Converts image to normalized torch tensor for inference."""
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) / 255.0
        img = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1)
        for i in range(3):
            img[i] = (img[i] - IMG_NORM_MEAN[i]) / IMG_NORM_STD[i]
        return img.unsqueeze(0).to(device)

    # --- Initialize camera ---
    pipeline = dai.Pipeline()
    cam = pipeline.createColorCamera()
    cam.setPreviewSize(CAM_WIDTH, CAM_HEIGHT)
    cam.setInterleaved(False); cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    # --- Joystick & motor ---
    js = init_joystick()
    motor = init_motor()
    print("Joystick + VESC ready. Hold RB to record, BACK+START to quit.")

    shared = {'rays': [1.0] * NUM_RAYS}
    lock = threading.Lock()
    acceleration = 0.0

    # --- Start recording session ---
    with dai.Device(pipeline) as device_cam, open("recorded_drive.csv", "w", newline="") as f:
        writer = csv.writer(f)
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

            # Exit shortcut
            if back and start:
                print("Exit requested.")
                break

            # Compute target acceleration from trigger values
            throttle = (rt + 1) / 6
            brake = (lt + 1) / 7
            target = max(-MAX_SPEED, min(MAX_SPEED, throttle - brake))

            # Smooth accel update
            if acceleration < target:
                acceleration = min(acceleration + DELTA_ACC, target)
            elif acceleration > target:
                acceleration = max(acceleration - DELTA_BRAKE, target)

            apply_motor_commands(motor, acceleration, steer)

            with lock:
                rays = shared['rays']
            print(f"Duty: {acceleration:.3f} | Steer: {(steer + 1) / 2:.3f} | RT: {rt:.2f} | LT: {lt:.2f}")

            # Save row on RB hold
            if rb:
                row = [acceleration, steer] + rays
                writer.writerow(row)
                print("[REC] Saved sample")

    # --- Cleanup ---
    motor.set_rpm(0)
    motor.stop_heartbeat()
    print("Shutdown complete.")

if __name__ == "__main__":
    main()
