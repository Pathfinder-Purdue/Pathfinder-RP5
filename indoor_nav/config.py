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
EMA_ALPHA    = 0.50    # EMA Constant:  0 = ignore new data, 1 = no smoothing
MIN_DWELL_MS = 400     # hold a command this long before allowing a change
HYSTERESIS   = 0.08    # extra margin to leave a triggered state


#  SENSOR FUSION WEIGHTS
# When all sensors are present these control the blend.
# W_LIDAR = 0.70    # most reliable distance measurement
# W_MIDAS = 0.30    # always-on depth estimate (software sensor)
W_LIDAR = 0.8
W_MIDAS = 0.2


#  LIDAR / TOF DISTANCE-TO-RISK MAPPING
DIST_NEAR_CENTER_M = 0.5   # metres — center zone: closer than this is max danger
DIST_FAR_CENTER_M  = 3     # metres — center zone: farther than this is safe
DIST_NEAR_SIDE_M   = 0.25   # metres — side zones: closer than this is max danger
DIST_FAR_SIDE_M    = 1.5     # metres — side zones: farther than this is safe


#  TOF GROUND-LEVEL OBSTACLE DETECTION  (deviation from calibrated baseline, mm)
TOF_NOISE_FLOOR_MM      = 20      # readings below this are sensor noise
TOF_DEVIATION_SAFE_MM   = 50      # deviation below this → safe (sensor noise)
TOF_DEVIATION_DANGER_MM = 300     # deviation above this → max risk
TOF_OVERRIDE_RISK       = 0.90    # if any ToF sector risk >= this, force STOP


#  ESP32 UART
ESP32_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0"
ESP32_BAUD = 115200


#  CALIBRATION (ToF + IMU, runs at startup)
CALIBRATION_SECS            = 5.0    # seconds to collect sensor samples
CALIBRATION_POLL_HZ         = 10     # how often to sample during calibration
CALIBRATION_PITCH_TOLERANCE = 15.0   # degrees — max pitch deviation from baseline
CALIBRATION_ROLL_TOLERANCE  = 15.0   # degrees — max roll deviation from baseline


#  TELEMETRY PANEL (visual display)
PANEL_HEIGHT = 160
