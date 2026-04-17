# TIDR Traceability — Pathfinder RP5

This document maps each Technical Interface Design Requirement to its implementation in the codebase and explains how to adjust or verify each one.

---

## File Overview

| File | Purpose |
|------|---------|
| `integrated.py` | Main entry point — starts all peripherals, runs the pipeline loop, displays overlays with live Hz readings |
| `indoor_nav/config.py` | All tuneable parameters — sensor weights, thresholds, intervals, UART settings |
| `indoor_nav/sensors.py` | Peripheral drivers — `ESP32Reader` (UART), persistent LiDAR, ToF/IMU/GPS parsing |
| `indoor_nav/models.py` | ML model loading and inference — YOLOv8 object detection, MiDaS depth estimation |
| `indoor_nav/detection.py` | Post-processing — depth normalization, sector risk extraction, YOLO obstacle scoring |
| `indoor_nav/risk_engine.py` | Sensor fusion — weighted blend of MiDaS + LiDAR + ToF into [L, C, R] risk scores, EMA smoothing |
| `indoor_nav/decision.py` | Navigation FSM — directional scoring, ToF ground validation, LiDAR flank validation |
| `indoor_nav/visualization.py` | Overlay rendering — depth colormap, YOLO bounding boxes (proximity-colored), telemetry panel |
| `haptic_zones.py` | LiDAR scan-to-zone mapping — converts raw scan data into 5 directional risk zones |

---

## TIDR 1 — Camera Image Read Rate (minimum 1 Hz)

**Where:** `integrated.py` — `capture_worker()` function reads frames in a dedicated thread via `cap.read()`. The `HzTracker` class measures actual capture rate and displays it on-screen as `Cam XX.XHz`.

**How to verify:** The camera Hz is measured live, not hardcoded. It reflects the true frame grab rate. To throttle the camera and prove the reading changes, add a `time.sleep()` call inside `capture_worker()` after `cap.read()`. For example, `time.sleep(1.0)` would produce ~1 Hz. Removing the sleep restores full speed. The displayed Hz will update accordingly.

**Config:** Camera device ID is `CAMERA_ID` in `indoor_nav/config.py`. Resolution is `FRAME_WIDTH` / `FRAME_HEIGHT`.

---

## TIDR 2 — Camera USB Connection

**Where:** `integrated.py` — `main()` opens the camera with `cv2.VideoCapture(CAMERA_ID)`. The webcam is physically connected via USB to the Raspberry Pi 5 and appears as `/dev/video0`.

**Physical verification:** The USB cable between the webcam and Pi 5 is visible. Unplugging it causes the camera init to fail with a clear error message.

---

## TIDR 3 — LiDAR Prioritized for Final Navigation Decisions

**Where:** This is implemented at two levels:

1. **Sensor fusion weights** in `indoor_nav/config.py`:
   - `W_LIDAR = 0.50` (highest weight)
   - `W_TOF = 0.30`
   - `W_MIDAS = 0.20` (lowest weight)

   These weights are used in `indoor_nav/risk_engine.py` — `fuse_sector_risks()`. When all sensors are present, LiDAR contributes 50% of the blended risk score. When a sensor is absent, its weight is redistributed proportionally, so LiDAR's share grows further.

2. **LiDAR flank validation** in `indoor_nav/decision.py` — `NavigationFSM._flank_clear()`. Before the system recommends veering left or right, it checks that the LiDAR's extended coverage zone (FL or FR flank, outside the camera FOV) is below `LIDAR_FLANK_SAFE`. If the flank is blocked, that direction is vetoed regardless of what CV says.

**How to verify:** Adjust `W_LIDAR`, `W_TOF`, `W_MIDAS` in `indoor_nav/config.py`. Setting `W_LIDAR = 1.0` and others to `0.0` makes the system purely LiDAR-driven. Changing `LIDAR_FLANK_SAFE` adjusts how aggressively the flank veto triggers.

---

## TIDR 4 — UART Communication with Microcontroller

**Where:** `indoor_nav/sensors.py` — `ESP32Reader` class.

- **Port and baud rate:** Configured in `indoor_nav/config.py` as `ESP32_PORT = "/dev/ttyAMA0"` and `ESP32_BAUD = 115200`.
- **RX (receive from ESP32):** `ESP32Reader._reader_loop()` calls `self._ser.read_until(b'\n')` to receive newline-terminated messages over UART RX.
- **TX (send to ESP32):** `ESP32Reader.write()` calls `self._ser.write()` to transmit commands back over UART TX.
- **Serial open:** `ESP32Reader.start()` opens the port with `serial.Serial(port=..., baudrate=...)`.

**Physical verification:** The UART connection uses GPIO pins on the Pi 5 header — TX (GPIO 14) and RX (GPIO 15) — wired to the ESP32's RX and TX respectively. The `/dev/ttyAMA0` device is the Pi's hardware UART.

**How to verify:** Change `ESP32_BAUD` in `indoor_nav/config.py` to a mismatched value (e.g., `9600`). The ESP32 data will appear garbled or missing, proving the baud rate is not hardcoded and the UART link is real.

---

## TIDR 5 — Receive Sensor Data from Microcontroller

**Where:** `indoor_nav/sensors.py` — `ESP32Reader._parse_message()`.

The ESP32 sends semicolon-delimited, newline-terminated messages:
```
Inc: N; TOF: v0,v1,...,v15; IMU: ax,ay,az,gx,gy,gz; GPS: v0,v1,v2\n
```

The parser extracts:
- **ToF** — 16 integer values (mm) from the 4x4 VL53L5CX sensor grid
- **IMU** — 6 float values (accelerometer + gyroscope)
- **GPS** — 3 float values

Each sensor type has a sequence counter (`tof_seq`, `imu_seq`, `gps_seq`) that increments on every new valid reading. The `HzTracker` in `integrated.py` uses these counters to compute and display the live update rate for each sensor (e.g., `ToF [8.5Hz]`).

**How to verify:** The Hz display is measured from actual incoming data, not hardcoded. Changing the ESP32 firmware's send interval directly changes the displayed Hz. On the Pi side, the receive polling interval is in `ESP32Reader._reader_loop()` — the `time.sleep(0.005)` idle wait can be adjusted to demonstrate responsiveness.

---

## TIDR 6 — Sensor Fusion for System Action

**Where:** The fusion pipeline spans three files:

1. **`indoor_nav/risk_engine.py`** — `fuse_sector_risks()` blends MiDaS depth, YOLO obstacles, LiDAR, and ToF into [L, C, R] risk scores using configurable weights. `RiskSmoother` applies EMA smoothing (controlled by `EMA_ALPHA` in config).

2. **`indoor_nav/decision.py`** — `NavigationFSM.update()` takes the fused [L, C, R] risks plus optional LiDAR flanks [FL, FR] and ToF ground risks [L, R]. It:
   - Scores three candidate directions by fused risk
   - Validates the safest direction against ToF ground clearance (`_tof_clear()`)
   - Validates against LiDAR flank clearance (`_flank_clear()`)
   - Outputs a command: GO, SLOW, VEER LEFT, VEER RIGHT, or STOP

3. **`indoor_nav/config.py`** — All fusion and decision thresholds:
   - `STOP_THRESHOLD` — risk level that triggers a full stop
   - `AVOID_THRESHOLD` — risk level that triggers veering
   - `CAUTION_THRESHOLD` — risk level that triggers slowing
   - `TOF_VETO_RISK` — ToF ground risk that vetoes a direction
   - `LIDAR_FLANK_SAFE` — LiDAR flank risk ceiling for veering
   - `EMA_ALPHA` — smoothing constant (higher = more responsive)

**How to verify:** All thresholds are in `indoor_nav/config.py`. Changing `STOP_THRESHOLD` from `0.80` to `0.30` makes the system stop earlier. Changing `W_MIDAS` / `W_LIDAR` / `W_TOF` shifts which sensors dominate the fused output. Changing `EMA_ALPHA` adjusts how quickly the system reacts to new readings.

---

## TIDR 7 — Microcontroller Sends Sensor Data to Onboard Computer

**Where:** This is the ESP32-side of TIDR 5. The ESP32 firmware transmits sensor readings over UART TX, which the Pi receives on UART RX via `ESP32Reader._reader_loop()` in `indoor_nav/sensors.py`.

**Physical verification:** The UART TX line from the ESP32 connects to the Pi's RX pin (GPIO 15). Data flow is visible in the overlay — if the ESP32 is disconnected, the ToF/IMU/GPS displays show `waiting ...` and their Hz drops to `0.0`.

---

## Frequency Verification Summary

All peripheral Hz values displayed on screen are **measured, not hardcoded**. The `HzTracker` class in `integrated.py` uses a sliding window of timestamps to compute actual throughput.

| Peripheral | Where Hz is ticked | How to change rate |
|------------|---|---|
| Camera | `capture_worker()` in `integrated.py` | Add `time.sleep(N)` after `cap.read()` |
| YOLO | `inference_worker()` in `integrated.py` | Change `YOLO_INTERVAL` in config |
| MiDaS | `inference_worker()` in `integrated.py` | Change `MIDAS_INTERVAL` in config |
| LiDAR | Main loop in `integrated.py` | LiDAR scan rate is hardware-driven |
| ToF | Seq counter in `ESP32Reader` | Change ESP32 firmware send rate |
| IMU | Seq counter in `ESP32Reader` | Change ESP32 firmware send rate |
| GPS | Seq counter in `ESP32Reader` | Change ESP32 firmware send rate |
