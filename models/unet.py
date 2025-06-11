"""
models/unet.py

Implementation of a simplified U-Net architecture for image segmentation.
Used to generate a binary mask from an RGB image (e.g., for autonomous driving).
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

# === DoubleConv ===
class DoubleConv(nn.Module):
    """
    Two consecutive convolutional layers with BatchNorm and ReLU.
    Structure: Conv2d -> BatchNorm -> ReLU -> Conv2d -> BatchNorm -> ReLU
    """
    def __init__(self, in_c, out_c, mid_c=None):
        super().__init__()
        if not mid_c:
            mid_c = out_c
        self.conv = nn.Sequential(
            nn.Conv2d(in_c, mid_c, 3, padding=1),
            nn.BatchNorm2d(mid_c),
            nn.ReLU(inplace=True),
            nn.Conv2d(mid_c, out_c, 3, padding=1),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.conv(x)

# === Down ===
class Down(nn.Module):
    """
    Downscaling block: max pooling followed by a DoubleConv.
    Reduces spatial dimensions while increasing depth.
    """
    def __init__(self, in_c, out_c):
        super().__init__()
        self.pool_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_c, out_c)
        )

    def forward(self, x):
        return self.pool_conv(x)

# === Up ===
class Up(nn.Module):
    """
    Upscaling block: upsampling followed by concatenation and DoubleConv.
    Uses bilinear interpolation to increase spatial size.
    Applies padding to align dimensions if necessary.
    """
    def __init__(self, in_c, out_c):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = DoubleConv(in_c, out_c, in_c // 2)

    def forward(self, x1, x2):
        x1 = self.up(x1)
        # Pad to match size of x2
        diffY = x2.size(2) - x1.size(2)
        diffX = x2.size(3) - x1.size(3)
        x1 = F.pad(x1, [diffX // 2, diffX - diffX // 2,
                        diffY // 2, diffY - diffY // 2])
        return self.conv(torch.cat([x2, x1], dim=1))

# === OutConv ===
class OutConv(nn.Module):
    """
    Final convolution layer that maps to the output class space.
    Typically used to output a single-channel mask.
    """
    def __init__(self, in_c, out_c):
        super().__init__()
        self.c = nn.Conv2d(in_c, out_c, 1)

    def forward(self, x):
        return self.c(x)

# === SimpleUNet ===
class SimpleUNet(nn.Module):
    """
    Simplified U-Net architecture with 4 down and 4 up blocks.
    Input: RGB image (3 channels)
    Output: Segmentation mask (1 channel)
    """
    def __init__(self, n_channels=3, n_classes=1):
        super().__init__()
        self.inc = DoubleConv(n_channels, 64)
        self.d1 = Down(64, 128)
        self.d2 = Down(128, 256)
        self.d3 = Down(256, 512)
        self.d4 = Down(512, 512)
        self.u1 = Up(1024, 256)
        self.u2 = Up(512, 128)
        self.u3 = Up(256, 64)
        self.u4 = Up(128, 64)
        self.outc = OutConv(64, n_classes)

    def forward(self, x):
        x1 = self.inc(x)
        x2 = self.d1(x1)
        x3 = self.d2(x2)
        x4 = self.d3(x3)
        x5 = self.d4(x4)
        x = self.u1(x5, x4)
        x = self.u2(x, x3)
        x = self.u3(x, x2)
        x = self.u4(x, x1)
        return self.outc(x)
