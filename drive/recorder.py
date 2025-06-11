# drive/recorder.py
"""
Record dataset samples from a Jetson + DepthAI setup,
using a trained mask model and gamepad input.
Run with: python3 recorder.py mask.pth
"""

import sys
import time
import csv
import signal
import depthai as dai
import numpy as np
import torch
import torch.nn as nn
import cv2
from math import sqrt
from drive.joystick_input import init_joystick, read_inputs, Input

# === Constants ===
CAM_WIDTH, CAM_HEIGHT = 320, 180
NUM_RAYS = 24
FOV = 180
MAX_DISTANCE = sqrt(CAM_WIDTH**2 + CAM_HEIGHT**2)
IMG_NORM_MEAN = [0.485, 0.456, 0.406]
IMG_NORM_STD = [0.229, 0.224, 0.225]

# === Model Definition ===
class DoubleConv(nn.Module):
    def __init__(self, in_c, out_c, mid_c=None):
        super().__init__()
        if not mid_c: mid_c = out_c
        self.conv = nn.Sequential(
            nn.Conv2d(in_c, mid_c, 3, padding=1), nn.BatchNorm2d(mid_c), nn.ReLU(True),
            nn.Conv2d(mid_c, out_c, 3, padding=1), nn.BatchNorm2d(out_c), nn.ReLU(True))
    def forward(self, x): return self.conv(x)

class Down(nn.Module):
    def __init__(self, in_c, out_c):
        super().__init__()
        self.pool_conv = nn.Sequential(nn.MaxPool2d(2), DoubleConv(in_c, out_c))
    def forward(self, x): return self.pool_conv(x)

class Up(nn.Module):
    def __init__(self, in_c, out_c):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = DoubleConv(in_c, out_c, in_c // 2)
    def forward(self, x1, x2):
        x1 = self.up(x1)
        dy = x2.size(2) - x1.size(2); dx = x2.size(3) - x1.size(3)
        x1 = nn.functional.pad(x1, [dx//2, dx-dx//2, dy//2, dy-dy//2])
        return self.conv(torch.cat([x2, x1], dim=1))

class OutConv(nn.Module):
    def __init__(self, in_c, out_c): super().__init__(); self.c = nn.Conv2d(in_c, out_c, 1)
    def forward(self, x): return self.c(x)

class SimpleUNet(nn.Module):
    def __init__(self, n_channels=3, n_classes=1):
        super().__init__()
        self.inc=DoubleConv(n_channels, 64); self.d1=Down(64,128); self.d2=Down(128,256)
        self.d3=Down(256,512); self.d4=Down(512, 512)
        self.u1=Up(1024,256); self.u2=Up(512,128); self.u3=Up(256,64); self.u4=Up(128,64)
        self.outc=OutConv(64, n_classes)
    def forward(self, x):
        x1=self.inc(x); x2=self.d1(x1); x3=self.d2(x2); x4=self.d3(x3); x5=self.d4(x4)
        x=self.u1(x5,x4); x=self.u2(x,x3); x=self.u3(x,x2); x=self.u4(x,x1)
        return self.outc(x)

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

# === Compute target acceleration ===
def compute_target(input_state, max_speed=0.15):
    throttle = (input_state[Input.RT] + 1) / 6
    brake = (input_state[Input.LT] + 1) / 7
    clamped_throttle = min(throttle, max_speed)
    clamped_brake = min(brake, -max_speed)
    return max(-max_speed, min(max_speed, clamped_throttle - clamped_brake))

# === Main ===
keep_running = True

def signal_handler(sig, frame):
    global keep_running
    print("\nSignal received. Exiting...")
    keep_running = False

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 recorder.py mask.pth")
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
    cam.setInterleaved(False); cam.setFps(30)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)

    js = init_joystick()
    input_state = [0 for _ in range(19)]
    input_state[Input.LT] = -1
    input_state[Input.RT] = -1
    print("Joystick ready:", js.get_name())
    print("[Start] Recording. Hold RB to record, BACK+START to quit.")

    with dai.Device(pipeline) as device_cam, open("recorded_drive.csv", "w", newline="") as f:
        writer = csv.writer(f)
        q_rgb = device_cam.getOutputQueue("rgb", 4, blocking=False)
        acceleration = 0.0
        DELTA_ACC = 0.003
        DELTA_BRAKE = 0.006

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
