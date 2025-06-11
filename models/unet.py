# models/unet.py
"""
U-Net model definition used for segmentation tasks in the Jetson-Prod project.
This model is imported by training, inference and recording scripts.
"""

import torch
import torch.nn as nn

class DoubleConv(nn.Module):
    """(conv => BN => ReLU) * 2"""
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

class Down(nn.Module):
    """Downscaling with maxpool then double conv"""
    def __init__(self, in_c, out_c):
        super().__init__()
        self.pool_conv = nn.Sequential(
            nn.MaxPool2d(2),
            DoubleConv(in_c, out_c)
        )

    def forward(self, x):
        return self.pool_conv(x)

class Up(nn.Module):
    """Upscaling then double conv"""
    def __init__(self, in_c, out_c):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = DoubleConv(in_c, out_c, in_c // 2)

    def forward(self, x1, x2):
        x1 = self.up(x1)
        dy = x2.size(2) - x1.size(2)
        dx = x2.size(3) - x1.size(3)
        x1 = nn.functional.pad(x1, [dx // 2, dx - dx // 2,
                                    dy // 2, dy - dy // 2])
        return self.conv(torch.cat([x2, x1], dim=1))

class OutConv(nn.Module):
    def __init__(self, in_c, out_c):
        super().__init__()
        self.conv = nn.Conv2d(in_c, out_c, kernel_size=1)

    def forward(self, x):
        return self.conv(x)

class SimpleUNet(nn.Module):
    """
    Simple U-Net architecture for binary segmentation.
    Input: 3-channel RGB image
    Output: 1-channel mask (logits)
    """
    def __init__(self, n_channels=3, n_classes=1):
        super().__init__()
        self.inc = DoubleConv(n_channels, 64)
        self.down1 = Down(64, 128)
        self.down2 = Down(128, 256)
        self.down3 = Down(256, 512)
        self.down4 = Down(512, 512)
        self.up1 = Up(1024, 256)
        self.up2 = Up(512, 128)
        self.up3 = Up(256, 64)
        self.up4 = Up(128, 64)
        self.outc = OutConv(64, n_classes)

    def forward(self, x):
        x1 = self.inc(x)
        x2 = self.down1(x1)
        x3 = self.down2(x2)
        x4 = self.down3(x3)
        x5 = self.down4(x4)
        x = self.up1(x5, x4)
        x = self.up2(x, x3)
        x = self.up3(x, x2)
        x = self.up4(x, x1)
        return self.outc(x)