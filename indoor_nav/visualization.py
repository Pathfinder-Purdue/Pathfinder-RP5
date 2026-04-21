"""All the drawing code. Boxes, depth overlay, telemetry panel."""

import cv2
import numpy as np

from indoor_nav.config import PANEL_HEIGHT


# color palette (BGR)
BG_DARK     = (30, 30, 30)
ACCENT      = (0, 200, 255)       # amber
GREEN       = (0, 230, 115)
RED         = (0, 60, 255)
WHITE       = (220, 220, 220)
GREY_DIM    = (100, 100, 100)
GREY_BORDER = (80, 80, 80)
FONT        = cv2.FONT_HERSHEY_SIMPLEX


def proximity_color(prox):
    """Smooth green to yellow to red gradient for proximity 0-1."""
    prox = max(0.0, min(1.0, prox))
    if prox < 0.5:
        t = prox * 2.0
        return (0, 230, int(115 + t * 115))
    else:
        t = (prox - 0.5) * 2.0
        return (0, int(230 - t * 170), int(230 + t * 25))


def command_color(command):
    """Color for a nav command."""
    if command == "STOP":
        return RED
    if command.startswith("VEER"):
        return ACCENT
    return GREEN


def _midas_depth_color(prox):
    """Color by MiDaS proximity: red (close), yellow (mid), green (far)."""
    prox = max(0.0, min(1.0, prox))
    if prox > 0.75:
        # close: red
        return (0, 60, 255)
    elif prox > 0.45:
        # moderate: yellow (interpolate red to yellow)
        t = (prox - 0.45) / 0.3
        return (0, int(200 - t * 140), int(255 - t * 0))
    else:
        # far: green
        return (0, 230, 115)


def draw_yolo_boxes(frame, yolo_results, yolo_obstacles, depth_normed=None):
    """Draw YOLO boxes colored by MiDaS depth (red=close, yellow=moderate, green=far)."""
    if yolo_results is None:
        return

    from indoor_nav.detection import box_proximity

    for box in yolo_results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        label = yolo_results.names[cls_id]

        prox = box_proximity(box, depth_normed)
        color = _midas_depth_color(prox)
        thickness = 2
        tag = f"YOLO: {label}, Confidence: {conf:.0%} | MiDaS: {prox:.0%}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(frame, tag, (x1, y1 - 6),
                    FONT, 0.42, color, 1, cv2.LINE_AA)


def draw_depth_overlay(frame, depth_map, alpha=0.25):
    """Blend MiDaS depth colormap onto the frame."""
    if depth_map is None:
        return
    d_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    d_color = cv2.applyColorMap(d_norm, cv2.COLORMAP_MAGMA)
    cv2.addWeighted(d_color, alpha, frame, 1 - alpha, 0, dst=frame)


def build_telemetry_panel(width, decision, sector_risks, fsm_state, fps, motor_values=None):
    """Info strip below video: state, sector risk + motor bars, nav arrow, FPS."""
    panel = np.full((PANEL_HEIGHT, width, 3), BG_DARK, dtype=np.uint8)

    if motor_values is None:
        motor_values = sector_risks

    # fixed-width columns tuned for 640 px video
    state_x = 10
    risk_x = 150
    motor_x = 350
    nav_x = 550

    _draw_state_column(panel, decision, fsm_state, x=state_x)
    _draw_value_bars(panel, "SECTOR", sector_risks, x=risk_x)
    _draw_value_bars(panel, "MOTOR", motor_values, x=motor_x)
    _draw_nav_cue(panel, decision, fps, x=nav_x)

    cv2.line(panel, (0, 0), (width, 0), ACCENT, 2)  # separator
    return panel


def _draw_state_column(panel, decision, fsm_state, x):
    """Left column: state + command."""
    cv2.putText(panel, "STATE", (x, 22),
                FONT, 0.40, ACCENT, 1, cv2.LINE_AA)
    cv2.putText(panel, fsm_state, (x, 46),
                FONT, 0.56, WHITE, 2, cv2.LINE_AA)

    color = command_color(decision.command)
    cv2.putText(panel, decision.command, (x, 74),
                FONT, 0.62, color, 2, cv2.LINE_AA)
    cv2.putText(panel, decision.explanation, (x, 98),
                FONT, 0.34, WHITE, 1, cv2.LINE_AA)


def _draw_value_bars(panel, title, values, x):
    """Draw a compact LB/L/C/R/RB bar stack for normalized values."""
    cv2.putText(panel, title, (x, 22),
                FONT, 0.40, ACCENT, 1, cv2.LINE_AA)

    labels  = ["LB", "L", "C", "R", "RB"]
    max_bar = 150

    for idx, (label, value) in enumerate(zip(labels, values)):
        value = max(0.0, min(1.0, float(value)))
        y = 38 + idx * 24
        bar_w = int(value * max_bar)
        bar_color = proximity_color(value)

        cv2.putText(panel, label, (x, y + 12),
                    FONT, 0.38, WHITE, 1, cv2.LINE_AA)
        cv2.rectangle(panel, (x + 18, y), (x + 18 + bar_w, y + 16),
                      bar_color, -1)
        cv2.rectangle(panel, (x + 18, y), (x + 18 + max_bar, y + 16),
                      GREY_BORDER, 1)
        cv2.putText(panel, f"{int(round(value * 100)):>3}%", (x + 86, y + 13),
                    FONT, 0.33, WHITE, 1, cv2.LINE_AA)


def _draw_nav_cue(panel, decision, fps, x):
    """Right column: direction arrow, urgency, FPS."""
    cv2.putText(panel, "NAV CUE", (x, 22),
                FONT, 0.40, ACCENT, 1, cv2.LINE_AA)

    cx, cy, length = x + 46, 68, 26

    if decision.command == "VEER LEFT":
        pts = np.array([
            [cx - length, cy],
            [cx + 8, cy - 14],
            [cx + 8, cy + 14],
        ], np.int32)
        cv2.fillPoly(panel, [pts], ACCENT)

    elif decision.command == "VEER RIGHT":
        pts = np.array([
            [cx + length, cy],
            [cx - 8, cy - 14],
            [cx - 8, cy + 14],
        ], np.int32)
        cv2.fillPoly(panel, [pts], ACCENT)

    elif decision.command == "GO":
        pts = np.array([
            [cx, cy - length],
            [cx - 14, cy + 8],
            [cx + 14, cy + 8],
        ], np.int32)
        cv2.fillPoly(panel, [pts], GREEN)

    else:
        cv2.circle(panel, (cx, cy), 18, RED, -1)
        cv2.putText(panel, "!", (cx - 5, cy + 7),
                    FONT, 0.6, WHITE, 2, cv2.LINE_AA)

    cv2.putText(panel, f"U: {decision.urgency:.0%}", (x, 108),
                FONT, 0.40, WHITE, 1, cv2.LINE_AA)
    cv2.putText(panel, f"FPS: {fps:.1f}", (x, 132),
                FONT, 0.40, GREEN, 1, cv2.LINE_AA)
