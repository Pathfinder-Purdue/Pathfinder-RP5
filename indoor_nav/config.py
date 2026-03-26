"""Tuneable parameter lives here (grouped by subsystem)"""

#  CAMERA
CAMERA_ID    = 0
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

#  INFERENCE SCHEDULING
YOLO_INTERVAL  = 2      # YOLO object detection every N frames
MIDAS_INTERVAL = 3      # MiDaS depth estimation every N frames


#  YOLO OBJECT DETECTION
YOLO_CONFIDENCE = 0.4   # minimum detection confidence

# COCO class IDs that count as navigation obstacles.
# Can add / remove obstacle IDs here later.
YOLO_OBSTACLE_CLASSES = {
    0,    # person
    15,   # bench
    24,   # backpack
    25,   # umbrella
    26,   # handbag
    28,   # suitcase
    56,   # chair
    57,   # couch
    58,   # potted plant
    59,   # bed
    60,   # dining table
    62,   # tv
    63,   # laptop
}


#  DEPTH / PROXIMITY THRESHOLDS  (MiDaS, normalized 0–1)
# MiDaS maps are normalized so 1.0 = closest to camera.
DEPTH_CLOSE_THRESHOLD = 0.55   # pixel fraction considered "close" in a sector

# Proximity zones for coloring YOLO bounding boxes:
PROX_NEAR = 0.70    # ≥ this = HIGH threat   (red)
PROX_MID  = 0.45    # ≥ this = MODERATE      (amber)
                     # < this = LOW threat    (green)

#  FSM DECISION THRESHOLDS  (risk score 0.0–1.0)
# Evaluated top-down: first condition met wins.
#   1. center risk ≥ STOP_THRESHOLD    = STOP
#   2. center risk ≥ AVOID_THRESHOLD   = VEER LEFT / RIGHT
#   3. max risk    ≥ CAUTION_THRESHOLD = SLOW
#   4. otherwise                       = GO
STOP_THRESHOLD    = 0.80
AVOID_THRESHOLD   = 0.55
CAUTION_THRESHOLD = 0.35


#  SMOOTHING & STABILITY
EMA_ALPHA    = 0.30    # EMA Constant:  0 = ignore new data, 1 = no smoothing
MIN_DWELL_MS = 400     # hold a command this long before allowing a change
HYSTERESIS   = 0.08    # extra margin to leave a triggered state


#  SENSOR FUSION WEIGHTS
# When all sensors are present these control the blend.
W_LIDAR = 0.50    # most reliable distance measurement
W_TOF   = 0.30    # short-range validation
W_MIDAS = 0.20    # always-on depth estimate (software sensor)


#  LIDAR / TOF DISTANCE-TO-RISK MAPPING
DIST_NEAR_M = 0.6     # metres — closer than this is max danger
DIST_FAR_M  = 2.5     # metres — farther than this is safe


#  TELEMETRY PANEL (visual display)
PANEL_HEIGHT = 160
