"""Helpers for loading and running:
  YOLOv8  (object detection)
  MiDaS   (monocular depth estimation)
"""

import cv2
import torch
import numpy as np

from indoor_nav.config import YOLO_CONFIDENCE


# Loading
def load_yolo():
    """Load YOLOv8-nano for real-time object detection."""
    from ultralytics import YOLO
    return YOLO("yolov8n.pt")


def load_midas(device):
    """Load MiDaS-small for monocular depth estimation."""
    model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
    model.to(device).eval()
    transform = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform
    return model, transform


# Inference
def run_yolo(model, frame):
    """Run YOLO on a single BGR frame."""
    results = model(frame, verbose=False, conf=YOLO_CONFIDENCE)
    return results[0] if results else None


def run_midas(model, transform, frame, device):
    """Run MiDaS on a single BGR frame."""
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_batch = transform(img_rgb).to(device)

    with torch.no_grad():
        depth = model(input_batch)
        depth = torch.nn.functional.interpolate(
            depth.unsqueeze(1),
            size=(frame.shape[0], frame.shape[1]),
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    return depth.cpu().numpy()
