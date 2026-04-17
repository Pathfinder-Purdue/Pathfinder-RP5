"""All the drawing code. Boxes, depth overlay, telemetry panel."""

import cv2
import numpy as np

from indoor_nav.config import PROX_NEAR, PROX_MID, PANEL_HEIGHT


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
    """Smooth green -> yellow -> red gradient based on proximity 0-1."""
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


def draw_yolo_boxes(frame, yolo_results, yolo_obstacles):
    """Draw YOLO boxes on frame. Obstacles get color-coded, everything else is grey."""
    if yolo_results is None:
        return

    # quick lookup for which boxes are obstacles
    obstacle_prox = {id(box): prox for _, prox, box in yolo_obstacles}

    for box in yolo_results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        label = yolo_results.names[cls_id]

        if id(box) in obstacle_prox:
            prox = obstacle_prox[id(box)]
            color = proximity_color(prox)
            thickness = 2
            tag = f"{label} {conf:.0%} [{prox:.0%}]"
        else:
            color = GREY_DIM
            thickness = 1
            tag = f"{label} {conf:.0%}"

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


def build_telemetry_panel(width, decision, risks, fsm_state, fps):
    """Info strip below the video feed: state, risk bars, nav arrow, FPS."""
    panel = np.full((PANEL_HEIGHT, width, 3), BG_DARK, dtype=np.uint8)

    _draw_state_column(panel, decision, fsm_state, x=12)
    _draw_risk_bars(panel, risks, x=width // 3)
    _draw_nav_cue(panel, decision, fps, x=2 * width // 3)

    cv2.line(panel, (0, 0), (width, 0), ACCENT, 2)  # separator
    return panel


def _draw_state_column(panel, decision, fsm_state, x):
    """Left column: state + command."""
    cv2.putText(panel, "STATE", (x, 22),
                FONT, 0.45, ACCENT, 1, cv2.LINE_AA)
    cv2.putText(panel, fsm_state, (x, 50),
                FONT, 0.65, WHITE, 2, cv2.LINE_AA)

    color = command_color(decision.command)
    cv2.putText(panel, decision.command, (x, 82),
                FONT, 0.75, color, 2, cv2.LINE_AA)
    cv2.putText(panel, decision.explanation, (x, 108),
                FONT, 0.40, WHITE, 1, cv2.LINE_AA)


def _draw_risk_bars(panel, risks, x):
    """Middle column: FL/L/C/R/FR risk bars."""
    cv2.putText(panel, "SECTOR RISK", (x, 22),
                FONT, 0.45, ACCENT, 1, cv2.LINE_AA)

    labels  = ["FL", "L", "C", "R", "FR"]
    max_bar = 150

    for idx, (label, value) in enumerate(zip(labels, risks)):
        y = 38 + idx * 24
        bar_w = int(value * max_bar)
        bar_color = proximity_color(value)

        cv2.putText(panel, label, (x, y + 12),
                    FONT, 0.42, WHITE, 1, cv2.LINE_AA)
        cv2.rectangle(panel, (x + 28, y), (x + 28 + bar_w, y + 16),
                      bar_color, -1)
        cv2.rectangle(panel, (x + 28, y), (x + 28 + max_bar, y + 16),
                      GREY_BORDER, 1)
        cv2.putText(panel, f"{value:.0%}", (x + 185, y + 13),
                    FONT, 0.36, WHITE, 1, cv2.LINE_AA)


def _draw_nav_cue(panel, decision, fps, x):
    """Right column: direction arrow, urgency, FPS."""
    cv2.putText(panel, "NAV CUE", (x, 22),
                FONT, 0.45, ACCENT, 1, cv2.LINE_AA)

    cx, cy, length = x + 70, 72, 35

    if decision.command == "VEER LEFT":
        pts = np.array([
            [cx - length, cy],
            [cx + 10, cy - 18],
            [cx + 10, cy + 18],
        ], np.int32)
        cv2.fillPoly(panel, [pts], ACCENT)

    elif decision.command == "VEER RIGHT":
        pts = np.array([
            [cx + length, cy],
            [cx - 10, cy - 18],
            [cx - 10, cy + 18],
        ], np.int32)
        cv2.fillPoly(panel, [pts], ACCENT)

    elif decision.command == "GO":
        pts = np.array([
            [cx, cy - length],
            [cx - 18, cy + 10],
            [cx + 18, cy + 10],
        ], np.int32)
        cv2.fillPoly(panel, [pts], GREEN)

    else:
        cv2.circle(panel, (cx, cy), 22, RED, -1)
        cv2.putText(panel, "!", (cx - 6, cy + 8),
                    FONT, 0.7, WHITE, 2, cv2.LINE_AA)

    cv2.putText(panel, f"Urgency: {decision.urgency:.0%}", (x, 120),
                FONT, 0.45, WHITE, 1, cv2.LINE_AA)
    cv2.putText(panel, f"FPS: {fps:.1f}", (x, 145),
                FONT, 0.45, GREEN, 1, cv2.LINE_AA)
