"""Combines YOLO boxes with MiDaS depth to get sector risks and per-obstacle scores."""

import cv2
import numpy as np

from indoor_nav.config import DEPTH_CLOSE_THRESHOLD, YOLO_OBSTACLE_CLASSES


def normalize_depth(depth_map):
    """Scale raw MiDaS depth to 0-1. Returns None if no depth yet."""
    if depth_map is None:
        return None
    return cv2.normalize(depth_map, None, 0.0, 1.0, cv2.NORM_MINMAX).astype(np.float64)


def midas_sector_risks(depth_normed):
    """Fraction of 'close' pixels in each sector [L, C, R]."""
    if depth_normed is None:
        return [0.0, 0.0, 0.0]

    h, w = depth_normed.shape
    third = w // 3
    sectors = [
        depth_normed[:, :third],           # left
        depth_normed[:, third:2 * third],  # centre
        depth_normed[:, 2 * third:],       # right
    ]
    return [float(np.mean(s > DEPTH_CLOSE_THRESHOLD)) for s in sectors]


def box_proximity(box, depth_normed):
    """Mean depth inside a YOLO box. Higher = closer = more dangerous."""
    if depth_normed is None:
        return 0.0

    h, w = depth_normed.shape
    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    if x2 <= x1 or y2 <= y1:
        return 0.0

    return float(np.mean(depth_normed[y1:y2, x1:x2]))


def scored_yolo_obstacles(yolo_results, depth_normed, frame_width):
    """Score each obstacle-class YOLO box by depth. Returns list of (sector, proximity, box)."""
    obstacles = []
    if yolo_results is None:
        return obstacles

    third = frame_width // 3

    for box in yolo_results.boxes:
        cls_id = int(box.cls[0])
        if cls_id not in YOLO_OBSTACLE_CLASSES:
            continue

        prox = box_proximity(box, depth_normed)
        x1, _, x2, _ = box.xyxy[0].tolist()
        centre_x = (x1 + x2) / 2
        sector = 0 if centre_x < third else (1 if centre_x < 2 * third else 2)

        obstacles.append((sector, prox, box))

    return obstacles
