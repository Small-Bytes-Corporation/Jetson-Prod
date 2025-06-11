import os
import sys
import time
import signal
import argparse
from pathlib import Path
from typing import Tuple, Dict, Optional, List
import random
import math

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torch.utils.data._utils.collate as torch_collate
import torchvision.transforms.v2 as transforms
from PIL import Image, ImageOps, ImageEnhance, ImageDraw
from tqdm import tqdm
import cv2

# --- Configuration ---
IMG_WIDTH, IMG_HEIGHT = 320, 180
BATCH_SIZE = 8
LEARNING_RATE = 1e-4
VAL_SPLIT, TEST_SPLIT = 0.05, 0.05
RANDOM_SEED = 42
NUM_WORKERS = 0
LOG_FREQ = 100
DEFAULT_MODEL_FILENAME = 'best-real-lines.pth'
DEFAULT_METRICS_FILENAME = 'metrics-real-lines.csv'

AUG_APPLY_PROB = 0.9
AUG_ANGLE_RANGE = (-45.0, 45.0)
AUG_SCALE_RANGE = (1.0, 1.5)
AUG_FLIP_PROB = 0.5
AUG_BRIGHTNESS_RANGE = (0.1, 2)
AUG_CONTRAST_RANGE = (0.1, 3)
AUG_MASK_BINARIZATION_THRESHOLD_RATIO = 0.1

DEBUG_SAVE_FREQ = 50
MAX_DEBUG_IMAGES = 9999

# --- Global Runtime State ---
stop_training = False
saved_debug_images_count = 0

def signal_handler(sig, frame):
    global stop_training
    stop_training = True
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# --- Helper Functions ---
def ensure_dir_exists(dir_path: Path, desc: str):
    if not dir_path.is_dir():
        sys.exit(f"Error: {desc} directory '{dir_path}' not found.")

def ensure_file_exists(file_path: Path, desc: str):
    if not file_path.is_file():
        sys.exit(f"Error: {desc} file '{file_path}' not found.")

def ensure_path_does_not_exist(path_obj: Path, desc: str):
    if path_obj.exists():
        sys.exit(f"Error: {desc} '{path_obj}' already exists. "
                 "Please remove or use a different path.")

def create_output_dir(dir_path: Path, desc: str):
    ensure_path_does_not_exist(dir_path, desc)
    dir_path.mkdir(parents=True, exist_ok=False)

def create_parent_dir_for_file(file_path: Path):
    file_path.parent.mkdir(parents=True, exist_ok=True)

# --- Augmentation Logic ---
def calculate_fill_scale(w: int, h: int, angle_deg: float) -> float:
    if angle_deg == 0: return 1.0
    rad = math.radians(abs(angle_deg)); c, s = math.cos(rad), math.sin(rad)
    return max((s * h + c * w) / w, (s * w + c * h) / h)

class PairedAugmentationTransform:
    def __init__(self, apply_prob: float = AUG_APPLY_PROB,
                 angle: Tuple[float,float]=AUG_ANGLE_RANGE,
                 scale: Tuple[float,float]=AUG_SCALE_RANGE,
                 flip_p: float = AUG_FLIP_PROB,
                 brightness: Tuple[float,float]=AUG_BRIGHTNESS_RANGE,
                 contrast: Tuple[float,float]=AUG_CONTRAST_RANGE,
                 mask_thresh_r: float=AUG_MASK_BINARIZATION_THRESHOLD_RATIO):
        self.apply_prob=apply_prob; self.angle_r=angle; self.scale_r=scale
        self.flip_p=flip_p; self.brightness_r=brightness; self.contrast_r=contrast
        self.mask_thresh = int(mask_thresh_r * 255)

    def _augment_single(self, img_pil: Image.Image, angle: float,
                        scale_f: float, flip: bool, is_mask: bool):
        w, h = img_pil.size
        resample = Image.Resampling.BILINEAR if is_mask else Image.Resampling.BICUBIC
        fill = 0 if is_mask else (0,0,0)

        rot_fill_s = calculate_fill_scale(w, h, angle)
        eff_scale = scale_f * rot_fill_s # scale_f is from AUG_SCALE_RANGE (>=1)
        scaled_w, scaled_h = int(w * eff_scale), int(h * eff_scale)

        aug_img = img_pil.resize((scaled_w, scaled_h), resample=resample)
        aug_img = aug_img.rotate(angle,resample=resample,expand=False,fillcolor=fill)

        curr_w, curr_h = aug_img.size
        left=(curr_w-w)//2; top=(curr_h-h)//2
        aug_img = aug_img.crop((left, top, left + w, top + h))
        if flip: aug_img = ImageOps.mirror(aug_img)
        return aug_img

    def __call__(self, img: Image.Image, mask: Image.Image):
        if random.random() > self.apply_prob: return img, mask, False # No aug

        angle = random.uniform(*self.angle_r)
        scale = random.uniform(*self.scale_r)
        flip = random.random() < self.flip_p
        bright_f = random.uniform(*self.brightness_r)
        contrast_f = random.uniform(*self.contrast_r)

        aug_img = self._augment_single(img, angle, scale, flip, False)
        if bright_f != 1.0:
            aug_img = ImageEnhance.Brightness(aug_img).enhance(bright_f)
        if contrast_f != 1.0:
            aug_img = ImageEnhance.Contrast(aug_img).enhance(contrast_f)

        aug_mask = self._augment_single(mask, angle, scale, flip, True)
        aug_mask = aug_mask.point(lambda p: 255 if p > self.mask_thresh else 0, 'L')
        return aug_img, aug_mask, True # Augmented

class TrackLimitDataset(Dataset):
    def __init__(self, image_dir: Path, mask_dir: Path, image_filenames: List[str],
                 image_transform=None, mask_transform=None,
                 paired_augment_transform: Optional[PairedAugmentationTransform] = None):
        self.image_dir = image_dir; self.mask_dir = mask_dir
        self.image_transform_fn = image_transform
        self.mask_transform_fn = mask_transform
        self.paired_augment_transform = paired_augment_transform
        mask_files = {p.name for p in mask_dir.glob('*.png')}
        self.images_meta = [(fn, Path(fn).stem) for fn in image_filenames if fn in mask_files]

        if not self.images_meta and image_filenames:
            print(f"Warn: No images in {image_dir} match masks in {mask_dir}.")
        elif not image_filenames: print(f"Warn: No image filenames for {image_dir}.")
        print(f"Dataset: {len(self.images_meta)} images from {image_dir}.")

    def __len__(self): return len(self.images_meta)

    def __getitem__(self, idx) -> Dict:
        img_name, _ = self.images_meta[idx]
        img_p = self.image_dir / img_name; mask_p = self.mask_dir / img_name
        img = Image.open(img_p).convert("RGB"); mask = Image.open(mask_p).convert("L")
        
        pil_for_debug_img, pil_for_debug_gt_mask = None, None
        was_augmented = False

        if self.paired_augment_transform:
            aug_img, aug_mask, was_augmented = self.paired_augment_transform(img, mask)
            if was_augmented:
                pil_for_debug_img = aug_img.copy()
                pil_for_debug_gt_mask = aug_mask.copy() # Store augmented GT mask
                img, mask = aug_img, aug_mask # Use augmented versions for tensor
            else: # Not augmented, but still need original PIL for debug if freq matches
                pil_for_debug_img = img.copy()
                pil_for_debug_gt_mask = mask.copy()


        tensor_img = self.image_transform_fn(img) if self.image_transform_fn else img
        tensor_mask = self.mask_transform_fn(mask) if self.mask_transform_fn else mask
        tensor_mask = torch.clamp(tensor_mask, 0.0, 1.0)

        item = {'image': tensor_img, 'mask': tensor_mask, 'original_filename': img_name}
        if pil_for_debug_img: item['pil_for_debug_image'] = pil_for_debug_img
        if pil_for_debug_gt_mask: item['pil_for_debug_gt_mask'] = pil_for_debug_gt_mask
        return item

# --- Model Definition (U-Net) ---
class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels, mid_channels=None):
        super().__init__()
        if not mid_channels: mid_channels = out_channels
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels,mid_channels,3,padding=1), nn.BatchNorm2d(mid_channels), nn.ReLU(True),
            nn.Conv2d(mid_channels,out_channels,3,padding=1), nn.BatchNorm2d(out_channels), nn.ReLU(True))
    def forward(self, x): return self.conv(x)

class Down(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.pool_conv = nn.Sequential(nn.MaxPool2d(2), DoubleConv(in_channels,out_channels))
    def forward(self, x): return self.pool_conv(x)

class Up(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.conv = DoubleConv(in_channels, out_channels, in_channels // 2)
    def forward(self, x1, x2):
        x1 = self.up(x1)
        dy = x2.size(2) - x1.size(2); dx = x2.size(3) - x1.size(3)
        x1 = nn.functional.pad(x1, [dx//2, dx-dx//2, dy//2, dy-dy//2])
        return self.conv(torch.cat([x2, x1], dim=1))

class OutConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__(); self.c=nn.Conv2d(in_channels,out_channels,1)
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

# --- Metrics ---
def _calc_metric_base(preds, targets, smooth=1e-6):
    preds_b = (torch.sigmoid(preds) > 0.5).float()
    targets_b = (targets > 0.5).float()
    dims = tuple(range(1, targets_b.ndim))
    intersection = (preds_b * targets_b).sum(dim=dims)
    return preds_b, targets_b, intersection, dims, smooth

def dice_coefficient(preds, targets, smooth=1e-6):
    p_b, t_b, inter, dims, sm = _calc_metric_base(preds, targets, smooth)
    dice = (2.*inter+sm) / (p_b.sum(dim=dims) + t_b.sum(dim=dims) + sm)
    return dice.mean()

def iou_coefficient(preds, targets, smooth=1e-6):
    p_b, t_b, inter, dims, sm = _calc_metric_base(preds, targets, smooth)
    union = (p_b.sum(dim=dims) + t_b.sum(dim=dims)) - inter
    iou = (inter + sm) / (union + sm)
    return iou.mean()

def pixel_accuracy(preds, targets):
    p_b, t_b, _, _, _ = _calc_metric_base(preds, targets)
    correct = (p_b == t_b).sum().float()
    return correct / t_b.numel() if t_b.numel() > 0 else 0.0

# --- Data Handling ---
def custom_collate_fn(batch: List[Dict]) -> Dict:
    if not batch: return {}
    elem = batch[0]; batched_output = {}
    for key in elem.keys():
        values = [d.get(key) for d in batch if d.get(key) is not None]
        if not values: continue
        if isinstance(values[0], Image.Image) or key == 'original_filename':
            batched_output[key] = values
        else:
            try: batched_output[key] = torch_collate.default_collate(values)
            except Exception: batched_output[key] = values
    return batched_output

def prepare_data(img_dir: Path, mask_dir: Path, apply_augs: bool = False):
    all_f = sorted([p.name for p in img_dir.glob('*.png') if p.is_file()])
    if not all_f: sys.exit(f"Error: No PNG images found in {img_dir}.")

    idxs = np.arange(len(all_f)); np.random.seed(RANDOM_SEED); np.random.shuffle(idxs)
    n_test=int(TEST_SPLIT*len(all_f)); n_val=int(VAL_SPLIT*(len(all_f)-n_test))
    tr_f=[all_f[i] for i in idxs[:len(all_f)-n_test-n_val]]
    val_f=[all_f[i] for i in idxs[len(all_f)-n_test-n_val : len(all_f)-n_test]]
    test_f=[all_f[i] for i in idxs[len(all_f)-n_test:]]
    print(f"Data split: Tr={len(tr_f)},Val={len(val_f)},Test={len(test_f)}")

    norm_m=[0.485,0.456,0.406]; norm_s=[0.229,0.224,0.225]
    img_tfm = transforms.Compose([
        transforms.Resize((IMG_HEIGHT,IMG_WIDTH)), transforms.ToImage(),
        transforms.ToDtype(torch.float32,scale=True),
        transforms.Normalize(mean=norm_m, std=norm_s)])
    mask_tfm = transforms.Compose([
        transforms.Resize((IMG_HEIGHT,IMG_WIDTH)), transforms.ToImage(),
        transforms.ToDtype(torch.float32,scale=True)])
    paired_aug = PairedAugmentationTransform() if apply_augs else None

    ds_args = {'image_transform':img_tfm, 'mask_transform':mask_tfm}
    tr_ds = TrackLimitDataset(img_dir,mask_dir,tr_f,**ds_args,paired_augment_transform=paired_aug)
    val_ds = TrackLimitDataset(img_dir,mask_dir,val_f,**ds_args)
    test_ds = TrackLimitDataset(img_dir,mask_dir,test_f,**ds_args)

    pin_mem = torch.cuda.is_available()
    eff_workers = NUM_WORKERS if NUM_WORKERS > 0 else max(1, (os.cpu_count() or 2)//2)
    dl_args = {'batch_size':BATCH_SIZE, 'num_workers':eff_workers,
               'pin_memory':pin_mem, 'persistent_workers':eff_workers>0}
    collate = custom_collate_fn if apply_augs else torch_collate.default_collate

    tr_loader = DataLoader(tr_ds, shuffle=True, **dl_args, collate_fn=collate)
    val_loader = DataLoader(val_ds, shuffle=False, **dl_args, collate_fn=torch_collate.default_collate)
    test_loader = DataLoader(test_ds, shuffle=False, **dl_args, collate_fn=torch_collate.default_collate)
    return tr_loader, val_loader, test_loader

# --- Training & Evaluation ---
def process_batch_metrics(outputs, targets, loss_val):
    acc = pixel_accuracy(outputs, targets).item()
    iou = iou_coefficient(outputs, targets).item()
    dice = dice_coefficient(outputs, targets).item()
    return {"loss":loss_val, "acc":acc, "iou":iou, "dice":dice}

def _create_overlay_pil(base_pil: Image.Image, mask_pil: Image.Image, color: tuple, alpha_override=None):
    overlay = base_pil.convert("RGBA")
    color_layer = Image.new("RGBA", base_pil.size)
    mask_px = mask_pil.load(); color_px = color_layer.load()
    
    for y in range(base_pil.height):
        for x in range(base_pil.width):
            if mask_px[x,y] > 0: # Mask assumed to be 0 or >0 (e.g. 255)
                alpha = int(mask_px[x,y] * (alpha_override if alpha_override else 1)) if not alpha_override else alpha_override
                alpha = max(0, min(255, alpha)) # Ensure alpha is 0-255
                current_color = color[:3] + (alpha,)
                color_px[x,y] = current_color
    return Image.alpha_composite(overlay, color_layer).convert("RGB")


def save_debug_training_image(batch_data, epoch, b_idx, dbg_dir, model_out_dev, gt_mask_dev):
    global saved_debug_images_count
    if not (dbg_dir and 'pil_for_debug_image' in batch_data and
            batch_data['pil_for_debug_image'] and
            (b_idx + 1) % DEBUG_SAVE_FREQ == 0 and
            saved_debug_images_count < MAX_DEBUG_IMAGES): return

    base_pil = batch_data['pil_for_debug_image'][0]
    orig_fname_stem = Path(batch_data['original_filename'][0]).stem
    if not base_pil: return
    
    dbg_dir.mkdir(parents=True, exist_ok=True) # Ensure dir exists

    # --- Simple Overlay ---
    pred_mask_simple_tensor = (torch.sigmoid(model_out_dev[0]) > 0.5).squeeze().cpu().byte()
    pred_mask_simple_pil = transforms.ToPILImage()(pred_mask_simple_tensor).convert('L')
    if pred_mask_simple_pil.size != base_pil.size:
        pred_mask_simple_pil = pred_mask_simple_pil.resize(base_pil.size, Image.NEAREST)
    
    simple_overlay_pil = _create_overlay_pil(base_pil, pred_mask_simple_pil, (255,0,0), 128) # Red, 50% alpha
    try:
        s_fname = f"{epoch}_{b_idx+1}_{orig_fname_stem}_simple.png"
        simple_overlay_pil.save(dbg_dir / s_fname)
    except Exception as e: print(f"Warn: Save simple debug img failed: {e}")

    # --- Detailed Overlay ---
    # Model prediction with dynamic alpha
    pred_conf_tensor = torch.sigmoid(model_out_dev[0]).squeeze().cpu() # Range 0-1
    # Convert confidence to an image where pixel values are 0-255 for alpha
    pred_alpha_mask_pil = transforms.ToPILImage()((pred_conf_tensor * 255).byte()).convert('L')
    if pred_alpha_mask_pil.size != base_pil.size:
        pred_alpha_mask_pil = pred_alpha_mask_pil.resize(base_pil.size, Image.NEAREST)

    details_base_pil = base_pil.copy().convert("RGBA")
    
    # Red layer for model prediction (dynamic alpha)
    red_layer = Image.new("RGBA", details_base_pil.size)
    red_px_access = red_layer.load()
    pred_alpha_px_access = pred_alpha_mask_pil.load()
    for y in range(details_base_pil.height):
        for x in range(details_base_pil.width):
            alpha_val = pred_alpha_px_access[x,y]
            if alpha_val > 10: # Only draw if somewhat confident
                 red_px_access[x,y] = (255,0,0, alpha_val)
    details_pil = Image.alpha_composite(details_base_pil, red_layer)

    # Green layer for ground truth (fixed 50% alpha)
    if 'pil_for_debug_gt_mask' in batch_data and batch_data['pil_for_debug_gt_mask']:
        gt_mask_pil = batch_data['pil_for_debug_gt_mask'][0]
        if gt_mask_pil.size != base_pil.size:
            gt_mask_pil = gt_mask_pil.resize(base_pil.size, Image.NEAREST)

        green_layer = Image.new("RGBA", details_base_pil.size)
        green_px_access = green_layer.load()
        gt_mask_px_access = gt_mask_pil.load()
        for y in range(details_base_pil.height):
            for x in range(details_base_pil.width):
                if gt_mask_px_access[x,y] > 0: # GT mask is 0 or 255
                    green_px_access[x,y] = (0,255,0, 128) # Green, 50% alpha
        details_pil = Image.alpha_composite(details_pil, green_layer)
    
    try:
        d_fname = f"{epoch}_{b_idx+1}_{orig_fname_stem}_details.png"
        details_pil.convert("RGB").save(dbg_dir / d_fname)
        saved_debug_images_count += 1 # Count pairs as one
    except Exception as e: print(f"Warn: Save detailed debug img failed: {e}")


def train_one_epoch(model, loader, opt, crit, dev, ep, met_fp, dbg_dir=None):
    global stop_training
    model.train()
    ep_mets = {k:0.0 for k in ["loss","acc","iou","dice"]}
    run_mets = ep_mets.copy(); b_count = 0; tot_b_proc = 0
    if not loader.dataset or len(loader.dataset)==0:
        print(f"Ep {ep}: Skip train, empty dataset."); return ep_mets

    prog_bar = tqdm(loader, desc=f"Ep {ep}")
    for b_idx, b_data in enumerate(prog_bar):
        if stop_training: break
        data = b_data['image'].to(dev)
        targets = b_data['mask'].to(dev).float()
        if targets.dim() == 3: targets = targets.unsqueeze(1)

        opt.zero_grad(); outputs = model(data)
        # Pass ground truth targets (from batch_data, already on device) for debug
        save_debug_training_image(b_data, ep, b_idx, dbg_dir, 
                                  outputs.detach().clone(), 
                                  targets.detach().clone()) # Pass GT targets

        if outputs.shape!=targets.shape:
            sys.exit(f"FATAL: Shape mismatch out={outputs.shape},tgt={targets.shape}")
        loss = crit(outputs, targets)
        if torch.isnan(loss): sys.exit("FATAL: NaN loss detected. Exiting.")
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        opt.step()

        with torch.no_grad():
            b_mets_vals = process_batch_metrics(outputs,targets,loss.item())
        for k in ep_mets: ep_mets[k]+=b_mets_vals[k]; run_mets[k]+=b_mets_vals[k]
        b_count += 1; tot_b_proc += 1

        if (b_idx + 1) % LOG_FREQ == 0 and b_count > 0:
            avg_mets = {k: v/b_count for k,v in run_mets.items()}
            met_fp.parent.mkdir(parents=True, exist_ok=True)
            hdr = not met_fp.exists() or met_fp.stat().st_size == 0
            with open(met_fp, 'a') as f:
                if hdr: f.write("epoch,step,train_loss,train_acc,train_iou,train_dice\n")
                f.write(f"{ep},{b_idx+1},{avg_mets['loss']:.6f},"
                        f"{avg_mets['acc']:.6f},{avg_mets['iou']:.6f},"
                        f"{avg_mets['dice']:.6f}\n")
            run_mets = {k:0.0 for k in run_mets}; b_count = 0
        prog_bar.set_postfix(**b_mets_vals)

    if tot_b_proc > 0:
        for k in ep_mets: ep_mets[k] /= tot_b_proc
    return ep_mets

def evaluate(model, loader, crit, dev, phase="val"):
    model.eval(); eval_mets={k:0.0 for k in ["loss","acc","iou","dice"]}
    tot_b = 0
    if not loader.dataset or len(loader.dataset)==0:
        print(f"Eval ({phase}): Skip, empty dataset."); return eval_mets
    with torch.no_grad():
        prog_bar = tqdm(loader, desc=f"Evaluating ({phase})")
        for b_data in prog_bar:
            if stop_training and phase == "val": break
            data = b_data['image'].to(dev)
            targets = b_data['mask'].to(dev).float()
            if targets.dim() == 3: targets = targets.unsqueeze(1)
            outputs = model(data)
            if outputs.shape!=targets.shape:
                sys.exit(f"FATAL ({phase}): Shape mismatch. Exiting.")
            loss = crit(outputs, targets)
            if torch.isnan(loss): sys.exit(f"FATAL ({phase}): NaN loss. Exiting.")

            b_m_disp = process_batch_metrics(outputs, targets, loss.item())
            for k in eval_mets: eval_mets[k] += b_m_disp[k]
            tot_b += 1
            prog_bar.set_postfix(**b_m_disp)
    if tot_b > 0:
        for k in eval_mets: eval_mets[k] /= tot_b
    return eval_mets

def _create_video_from_images_cv2(img_paths: List[str], out_vid_p: str, fps=30):
    if not img_paths: print("No debug images for video."); return
    try:
        first_img = cv2.imread(img_paths[0])
        if first_img is None: print(f"Err: Read failed {img_paths[0]}"); return
        h, w, _ = first_img.shape; fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(out_vid_p, fourcc, fps, (w,h))
        if not writer.isOpened(): print(f"Err: Open writer {out_vid_p}"); return
        for img_p_str in tqdm(img_paths, desc="Creating video"):
            img = cv2.imread(img_p_str)
            if img is None: print(f"Warn: Skip unreadable {img_p_str}"); continue
            writer.write(img)
        writer.release(); print(f"Video saved to {out_vid_p}")
    except Exception as e: print(f"Error creating video: {e}")

def convert_image(in_p: Path, out_p: Path, model_p: Path, dev,
                  dbg_dir: Optional[Path]=None, vid_p: Optional[Path]=None):
    ensure_file_exists(model_p, "Model for conversion")
    model = SimpleUNet(n_channels=3, n_classes=1)
    model.load_state_dict(torch.load(str(model_p), map_location=dev))
    model.to(dev); model.eval()

    norm_m=[0.485,0.456,0.406]; norm_s=[0.229,0.224,0.225]
    img_tfm_conv = transforms.Compose([
        transforms.Resize((IMG_HEIGHT,IMG_WIDTH)), transforms.ToImage(),
        transforms.ToDtype(torch.float32,scale=True),
        transforms.Normalize(mean=norm_m,std=norm_s)])
    dbg_frames_paths = []

    def process_single(in_f: Path, out_mask_f: Path):
        try:
            orig_pil = Image.open(in_f).convert("RGB"); orig_sz = orig_pil.size
            img_tensor = img_tfm_conv(orig_pil).unsqueeze(0).to(dev)
            
            pred_pil_raw_confidence = None # For detailed debug
            pred_pil_binary = None       # For simple debug and main output

            with torch.no_grad():
                out_tensor = model(img_tensor) # Shape (1, 1, H, W)
                sigmoid_out = torch.sigmoid(out_tensor)
                
                # For detailed debug (dynamic alpha)
                pred_pil_raw_confidence = transforms.ToPILImage()(sigmoid_out.squeeze().cpu()).convert('L')
                if pred_pil_raw_confidence.size != orig_sz:
                     pred_pil_raw_confidence = pred_pil_raw_confidence.resize(orig_sz, Image.Resampling.BILINEAR)


                # For simple debug and main output (binary)
                pred_np_binary = (sigmoid_out > 0.5).squeeze().cpu().numpy().astype(np.uint8)*255
                pred_pil_binary = Image.fromarray(pred_np_binary,'L').resize(orig_sz,Image.NEAREST)

            pred_pil_binary.save(out_mask_f) # Save main binary mask output

            if dbg_dir:
                dbg_dir.mkdir(parents=True, exist_ok=True)
                orig_fname_stem = in_f.stem

                # Simple overlay
                simple_overlay_pil = _create_overlay_pil(orig_pil, pred_pil_binary, (255,0,0), 128)
                simple_fname = dbg_dir / f"{orig_fname_stem}_simple.png"
                simple_overlay_pil.save(simple_fname)
                dbg_frames_paths.append(str(simple_fname)) # Add simple for video

                # Detailed overlay (model confidence only for convert)
                details_base_pil = orig_pil.copy().convert("RGBA")
                red_layer = Image.new("RGBA", details_base_pil.size)
                red_px = red_layer.load()
                conf_px = pred_pil_raw_confidence.load() # This is already 0-255 from ToPILImage
                for y in range(details_base_pil.height):
                    for x in range(details_base_pil.width):
                        alpha_val = conf_px[x,y]
                        if alpha_val > 10: # Threshold to reduce noise
                            red_px[x,y] = (255,0,0, alpha_val)

                final_details_pil = Image.alpha_composite(details_base_pil, red_layer).convert("RGB")
                details_fname = dbg_dir / f"{orig_fname_stem}_details.png"
                final_details_pil.save(details_fname)
            return True
        except Exception as e: print(f"Error processing {in_f.name}: {e}"); return False

    if in_p.is_dir():
        create_output_dir(out_p, "Output mask directory")
        if dbg_dir: create_output_dir(dbg_dir, "Convert debug directory")
        img_exts={'.png','.jpg','.jpeg'}
        img_files=[p for p in in_p.iterdir() if p.is_file() and p.suffix.lower() in img_exts]
        if not img_files: print(f"No images in {in_p}"); return False
        s_count = sum(process_single(f, out_p/f.name) for f in tqdm(img_files, desc="Converting"))
        print(f"Converted {s_count}/{len(img_files)}. Masks in '{out_p}'.")
        if dbg_dir: print(f"Debug images in '{dbg_dir}'.")
        if vid_p and dbg_frames_paths:
            dbg_frames_paths.sort()
            _create_video_from_images_cv2(dbg_frames_paths, str(vid_p))
        return s_count > 0
    else:
        ensure_file_exists(in_p, "Input image")
        create_parent_dir_for_file(out_p)
        ensure_path_does_not_exist(out_p, "Output mask file")
        if dbg_dir: create_output_dir(dbg_dir, "Convert debug directory")
        if process_single(in_p, out_p):
            print(f"Mask saved to {out_p}")
            if dbg_dir: print(f"Debug image in '{dbg_dir}'.")
            return True
        return False

def train_model(in_dir:Path, masks_dir:Path, model_sp:Path, met_fp:Path, dbg_dir=None):
    global stop_training, saved_debug_images_count
    saved_debug_images_count = 0
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    is_multi_gpu = torch.cuda.is_available() and torch.cuda.device_count() > 1
    print(f"Device: {dev}" + (f" ({torch.cuda.device_count()} GPUs)" if is_multi_gpu else ""))
    torch.manual_seed(RANDOM_SEED); np.random.seed(RANDOM_SEED)
    if torch.cuda.is_available(): torch.cuda.manual_seed_all(RANDOM_SEED)

    tr_load, val_load, test_load = prepare_data(in_dir, masks_dir, apply_augs=True)
    if len(tr_load.dataset)==0: sys.exit("FATAL: Train dataset empty. Exiting.")

    model = SimpleUNet(n_channels=3, n_classes=1)
    if model_sp.exists():
        try:
            model.load_state_dict(torch.load(str(model_sp), map_location=dev))
            print(f"Loaded existing model from {model_sp}")
        except Exception as e: print(f"Warn: Load model err: {e}. New model.")
    if is_multi_gpu: model = nn.DataParallel(model)
    model.to(dev)

    crit = nn.BCEWithLogitsLoss(); opt = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    best_val_iou = -1.0; start_tm = time.time()

    for ep in range(1, 9999):
        if stop_training: print(f"Ep {ep}: Stop signal, interrupting."); break
        tr_mets = train_one_epoch(model,tr_load,opt,crit,dev,ep,met_fp,dbg_dir)
        print(f"Ep {ep} Tr: L={tr_mets['loss']:.3f} A={tr_mets['acc']:.3f} "
              f"IoU={tr_mets['iou']:.3f} Dice={tr_mets['dice']:.3f}")
        if stop_training: print(f"Ep {ep}: Stop signal before val."); break

        if len(val_load.dataset) > 0:
            val_mets = evaluate(model, val_load, crit, dev, phase="val")
            print(f"Ep {ep} Val: L={val_mets['loss']:.3f} A={val_mets['acc']:.3f} "
                  f"IoU={val_mets['iou']:.3f} Dice={val_mets['dice']:.3f}")
            if val_mets['iou'] > best_val_iou:
                best_val_iou = val_mets['iou']
                m_state = model.module.state_dict() if is_multi_gpu else model.state_dict()
                model_sp.parent.mkdir(parents=True, exist_ok=True)
                torch.save(m_state, str(model_sp))
                print(f"Saved best model (IoU: {best_val_iou:.4f}) to {model_sp}")
        else:
            print(f"Ep {ep}: No val set. Saving model from this epoch.")
            m_state = model.module.state_dict() if is_multi_gpu else model.state_dict()
            model_sp.parent.mkdir(parents=True, exist_ok=True)
            torch.save(m_state, str(model_sp))
            print(f"Saved model (ep {ep}, no val) to {model_sp}")

    el_time_m = (time.time() - start_tm) / 60
    print(f"Training finished in {el_time_m:.2f} minutes"); return test_load

def test_model(model_p:Path, dev, in_dir:Path, masks_dir:Path, test_load=None):
    ensure_file_exists(model_p, "Model for testing")
    if test_load is None:
        print("Test loader not provided, preparing data for testing...")
        _, _, test_load = prepare_data(in_dir, masks_dir, apply_augs=False)
    if len(test_load.dataset)==0: print("Test dataset empty. Skip test."); return

    model = SimpleUNet(n_channels=3, n_classes=1)
    model.load_state_dict(torch.load(str(model_p), map_location=dev)); model.to(dev)
    crit = nn.BCEWithLogitsLoss()
    test_mets = evaluate(model, test_load, crit, dev, phase="test")
    print(f"--- Test Results ---\n Model: {model_p}\n "
          f"Loss: {test_mets['loss']:.4f}\n Acc : {test_mets['acc']:.4f}\n "
          f"IoU : {test_mets['iou']:.4f}\n Dice: {test_mets['dice']:.4f}")

if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    DEF_MODEL_P = script_dir / DEFAULT_MODEL_FILENAME
    DEF_METRICS_P = script_dir / DEFAULT_METRICS_FILENAME

    parser = argparse.ArgumentParser(description="U-Net Segmentation")
    subparsers = parser.add_subparsers(dest="command", required=True,
                                       help="train, test, convert")

    tr_p = subparsers.add_parser("train", help="Train model")
    tr_p.add_argument("input_dir", type=Path, help="Input image dir")
    tr_p.add_argument("masks_dir", type=Path, help="Mask label dir")
    tr_p.add_argument("--model",type=Path,default=DEF_MODEL_P,help=f"Save/load model ({DEF_MODEL_P})")
    tr_p.add_argument("--metrics",type=Path,default=DEF_METRICS_P,help=f"Save metrics ({DEF_METRICS_P})")
    tr_p.add_argument("--debug",type=Path,default=None,help="Save augmented train samples")

    test_p = subparsers.add_parser("test", help="Test model")
    test_p.add_argument("input_dir", type=Path, help="Input image dir")
    test_p.add_argument("masks_dir", type=Path, help="Mask label dir")
    test_p.add_argument("--model",type=Path,default=DEF_MODEL_P,help=f"Load model ({DEF_MODEL_P})")

    conv_p = subparsers.add_parser("convert", help="Convert image(s) to mask(s)")
    conv_p.add_argument("input_path", type=Path, help="Input image/dir path")
    conv_p.add_argument("output_path", type=Path, help="Output mask/dir path")
    conv_p.add_argument("--model",type=Path,default=DEF_MODEL_P,help=f"Model path ({DEF_MODEL_P})")
    conv_p.add_argument("--debug",type=Path,default=None,help="Save convert debug images")
    conv_p.add_argument("--video",type=Path,default=None,help="Video from debug frames (needs --debug)")

    args = parser.parse_args()
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    if args.command == "train":
        ensure_dir_exists(args.input_dir, "Training input")
        ensure_dir_exists(args.masks_dir, "Training masks")
        print(f"Train: In='{args.input_dir}', Masks='{args.masks_dir}'")
        print(f"Model='{args.model}', Metrics='{args.metrics}'")
        if args.debug:
            args.debug.mkdir(parents=True, exist_ok=True)
            print(f"Augmented debug samples to: '{args.debug}'")
        final_test_loader = train_model(args.input_dir,
            args.masks_dir, args.model, args.metrics, args.debug)
        if not stop_training and final_test_loader and len(final_test_loader.dataset) > 0:
            print("\n--- Train Complete, Evaluating on Test Set ---")
            test_model(args.model, dev, args.input_dir, args.masks_dir, final_test_loader)
        elif stop_training: print("Train interrupted. Skip final test.")
        else: print("Test loader empty/train issues. Skip final test.")

    elif args.command == "test":
        ensure_dir_exists(args.input_dir, "Test input")
        ensure_dir_exists(args.masks_dir, "Test masks")
        ensure_file_exists(args.model, "Test model")
        print(f"Test: In='{args.input_dir}', Masks='{args.masks_dir}', Model='{args.model}'")
        test_model(args.model, dev, args.input_dir, args.masks_dir)

    elif args.command == "convert":
        ensure_file_exists(args.model, "Convert model")
        if args.video:
            if not args.debug: sys.exit("Error: --video requires --debug.")
            if args.input_path.is_file(): sys.exit("Error: --video for dir input.")
            ensure_path_does_not_exist(args.video, "Output video file")
            create_parent_dir_for_file(args.video)

        print(f"Convert: In='{args.input_path}', Out='{args.output_path}', Model='{args.model}'")
        if args.debug: print(f"Convert debug images to: '{args.debug}'")
        if args.video: print(f"Convert debug video to: '{args.video}'")
        convert_image(args.input_path, args.output_path, args.model, dev, args.debug, args.video)

    print("Operation finished.")
