import argparse
import subprocess
import sys
import time
from collections import deque
from queue import Queue, Empty
from threading import Event, Lock, Thread

import cv2
import numpy as np
import torch

from indoor_nav.config import (
    CAMERA_ID, FRAME_WIDTH, FRAME_HEIGHT,
    YOLO_INTERVAL, MIDAS_INTERVAL,
)
from indoor_nav.models import load_yolo, load_midas, run_yolo, run_midas
from indoor_nav.sensors import (
    ESP32Reader, read_tof_ground_risk, read_imu,
    init_lidar, stop_lidar, read_lidar_sectors,
    calibrate_posture,
)
from indoor_nav.detection import normalize_depth, midas_sector_risks, scored_yolo_obstacles
from indoor_nav.risk_engine import fuse_sector_risks, RiskSmoother
from indoor_nav.decision import NavigationFSM
from indoor_nav.visualization import (
    draw_depth_overlay,
    draw_yolo_boxes,
    build_telemetry_panel,
)
from haptic_zones import HapticOutputLimiter


# Text-to-speech for navigation decisions

_SECTOR_NAMES = ["to your left", "ahead", "to your right"]


def _top_threat_phrase(yolo_obstacles, class_names):
    """Return e.g. 'Person ahead' for the closest YOLO obstacle, or None."""
    if not yolo_obstacles:
        return None
    # highest proximity = closest threat
    sector, prox, box = max(yolo_obstacles, key=lambda o: o[1])
    if prox < 0.3:
        return None
    label = class_names.get(int(box.cls[0]), "obstacle").capitalize()
    return f"{label} {_SECTOR_NAMES[sector]}"


class Speaker:
    """Non-blocking TTS using espeak-ng on a background thread."""
    def __init__(self, rate=160):
        self._queue = Queue(maxsize=1)
        self._rate = rate
        self._thread = None

    def start(self):
        self._thread = Thread(target=self._worker, name="tts-speaker", daemon=True)
        self._thread.start()
        print("[integrated] TTS speaker ready (espeak-ng)")

    def say(self, text):
        """Queue a phrase. Drops old queued phrase if not yet spoken."""
        try:
            self._queue.get_nowait()
        except Empty:
            pass
        self._queue.put(text)

    def _worker(self):
        while True:
            text = self._queue.get()
            try:
                subprocess.run(
                    ["espeak-ng", "-s", str(self._rate), "-a", "200", text],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass


# Frequency tracker

class HzTracker:
    """Measures update frequency using a sliding window."""
    def __init__(self, window=30):
        self._times = deque(maxlen=window)

    def tick(self):
        self._times.append(time.time())

    @property
    def hz(self):
        if len(self._times) < 2:
            return 0.0
        dt = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / dt if dt > 0 else 0.0


# ESP32 status overlay

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_GREEN = (0, 230, 115)
_RED   = (0, 60, 255)
_WHITE = (220, 220, 220)


def _draw_lidar_status(frame, lidar_5, hz):
    """Draw LiDAR [FL, L, C, R, FR] risk values in the top-right corner."""
    x = frame.shape[1] - 310
    y = 18
    if lidar_5 is not None:
        peak = max(lidar_5)
        color = _RED if peak > 0.7 else _GREEN
        cv2.putText(frame,
                    f"LiDAR [{hz:.1f}Hz]: FL={lidar_5[0]:.0%} L={lidar_5[1]:.0%} C={lidar_5[2]:.0%} R={lidar_5[3]:.0%} FR={lidar_5[4]:.0%}",
                    (x, y), _FONT, 0.34, color, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, f"LiDAR [{hz:.1f}Hz]: waiting ...",
                    (x, y), _FONT, 0.38, _RED, 1, cv2.LINE_AA)


def _draw_esp32_status(frame, tof_risks, imu_data, gps_data, hz_tof, hz_imu, hz_gps):
    """Draw ESP32 sensor values in the top-right corner of the frame."""
    x = frame.shape[1] - 310
    y = 36

    # ToF risk [L, R]
    if tof_risks is not None:
        peak = max(tof_risks)
        color = _RED if peak > 0.7 else _GREEN
        cv2.putText(frame,
                    f"ToF [{hz_tof:.1f}Hz]: L={tof_risks[0]:.0%}  R={tof_risks[1]:.0%}",
                    (x, y), _FONT, 0.38, color, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, f"ToF [{hz_tof:.1f}Hz]: waiting ...",
                    (x, y), _FONT, 0.38, _RED, 1, cv2.LINE_AA)

    # IMU [ax, ay, az, gx, gy, gz]
    y += 18
    if imu_data is not None:
        cv2.putText(frame,
                    f"IMU [{hz_imu:.1f}Hz]: {imu_data[0]:+6.1f} {imu_data[1]:+6.1f} {imu_data[2]:+6.1f}",
                    (x, y), _FONT, 0.38, _WHITE, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, f"IMU [{hz_imu:.1f}Hz]: waiting ...",
                    (x, y), _FONT, 0.38, _RED, 1, cv2.LINE_AA)

    # GPS
    y += 18
    if gps_data is not None:
        cv2.putText(frame,
                    f"GPS [{hz_gps:.1f}Hz]: {gps_data[1]:.5f}, {gps_data[2]:.5f}",
                    (x, y), _FONT, 0.38, _WHITE, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, f"GPS [{hz_gps:.1f}Hz]: waiting ...",
                    (x, y), _FONT, 0.38, _RED, 1, cv2.LINE_AA)


def _draw_pipeline_status(frame, hz_cam, hz_yolo, hz_midas):
    """Draw Camera / YOLO / MiDaS Hz on the frame."""
    x = frame.shape[1] - 310
    y = 90
    cv2.putText(frame,
                f"Cam {hz_cam:.1f}Hz | YOLO {hz_yolo:.1f}Hz | MiDaS {hz_midas:.1f}Hz",
                (x, y), _FONT, 0.38, _WHITE, 1, cv2.LINE_AA)


# Main

def main():
    parser = argparse.ArgumentParser(description="Pathfinder integrated pipeline")
    parser.add_argument("--viz", action="store_true", help="Enable OpenCV visualization")
    args = parser.parse_args()
    if args.viz:
        print("[integrated] Visualization enabled")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[integrated] torch device: {device}")

    # Load ML models
    print("[integrated] Loading YOLO ...")
    yolo_model = load_yolo()
    print("[integrated] Loading MiDaS ...")
    midas_model, midas_transform = load_midas(device)
    print("[integrated] Models ready.")

    # Open camera
    print("[integrated] Opening camera ...")
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        sys.exit("[integrated] ERROR -- could not open camera")
    for _ in range(10):
        cap.read()
    print("[integrated] Camera ready.")

    # Start LiDAR
    print("[integrated] Starting LiDAR ...")
    lidar_ok = init_lidar()
    if not lidar_ok:
        print("[integrated] WARNING -- LiDAR unavailable")

    # Start ESP32 UART reader
    print("[integrated] Starting ESP32 reader ...")
    esp32 = ESP32Reader()
    esp32_ok = esp32.start()
    if not esp32_ok:
        print("[integrated] WARNING -- ESP32 unavailable, ToF/IMU/GPS disabled")
        esp32 = None

    # Pipeline state
    fsm      = NavigationFSM()
    smoother = RiskSmoother()
    risks    = [0.0, 0.0, 0.0]

    lidar_5      = None
    lidar_inner  = None
    lidar_flanks = None
    tof_risks    = None
    imu_data     = None
    gps_data     = None

    # Start TTS speaker
    speaker = Speaker()
    speaker.start()
    last_spoken_command = None

    # Posture calibration (IMU-based, required for ToF ground detection)
    print("[integrated] Running posture calibration ...")
    calibration = calibrate_posture(speaker, esp32)
    if calibration is not None:
        print(f"[integrated] Calibration baseline: {calibration}")
    else:
        print("[integrated] WARNING -- calibration failed, using defaults")

    fps_buf = deque(maxlen=30)
    t_prev  = time.time()

    # Hz trackers for each peripheral
    hz_cam   = HzTracker()
    hz_yolo  = HzTracker()
    hz_midas = HzTracker()
    hz_lidar = HzTracker()
    hz_tof   = HzTracker()
    hz_imu   = HzTracker()
    hz_gps   = HzTracker()

    # Seq counters to detect new ESP32 data
    prev_tof_seq = 0
    prev_imu_seq = 0
    prev_gps_seq = 0

    shared = {
        "latest_frame":  None,
        "yolo_results":  None,
        "depth_map":     None,
        "capture_count": 0,
    }
    data_lock  = Lock()
    stop_event = Event()

    # Camera capture thread
    def capture_worker():
        """Continuously grab frames so capture is not blocked by inference."""
        consecutive_fails = 0
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                consecutive_fails += 1
                if consecutive_fails >= 100:
                    print("[integrated] Camera: too many read failures, stopping")
                    stop_event.set()
                    return
                time.sleep(0.01)
                continue
            consecutive_fails = 0
            hz_cam.tick()
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            with data_lock:
                shared["latest_frame"] = frame
                shared["capture_count"] += 1

    # Inference thread (YOLO + MiDaS)
    def inference_worker():
        """Run YOLO / MiDaS on latest frame with staggered scheduling."""
        last_seen = -1
        while not stop_event.is_set():
            with data_lock:
                frame = shared["latest_frame"]
                count = shared["capture_count"]
            if frame is None or count == last_seen:
                time.sleep(0.001)
                continue
            last_seen = count

            yolo_out = depth_out = None
            if count % YOLO_INTERVAL == 0:
                yolo_out = run_yolo(yolo_model, frame)
                hz_yolo.tick()
            if count % MIDAS_INTERVAL == 0:
                depth_out = run_midas(midas_model, midas_transform, frame, device)
                hz_midas.tick()

            if yolo_out is not None or depth_out is not None:
                with data_lock:
                    if yolo_out is not None:
                        shared["yolo_results"] = yolo_out
                    if depth_out is not None:
                        shared["depth_map"] = depth_out

    # Launch threads
    t_cap = Thread(target=capture_worker,  name="capture-worker",  daemon=True)
    t_inf = Thread(target=inference_worker, name="inference-worker", daemon=True)
    t_cap.start()
    t_inf.start()
    print("[integrated] Streaming -- press 'q' to quit")

    motor_limiter = HapticOutputLimiter(num_zones=3)

    # Main frame loop
    while not stop_event.is_set():
        # Grab latest frame + inference results
        with data_lock:
            frame = (None if shared["latest_frame"] is None
                     else shared["latest_frame"].copy())
            yolo_results = shared["yolo_results"]
            depth_map    = shared["depth_map"]

        if frame is None:
            time.sleep(0.001)
            continue

        # Read LiDAR sectors
        lidar_5 = read_lidar_sectors()                 # [FL,L,C,R,FR] 0-1 risk
        if lidar_5 is not None:
            hz_lidar.tick()
            lidar_inner  = lidar_5[1:4]                # [L,C,R] for fusion
            lidar_flanks = [lidar_5[0], lidar_5[4]]    # [FL,FR] for validation
        else:
            lidar_inner  = None
            lidar_flanks = None

        # Read ESP32 sensors — tick Hz on new data
        tof_risks = read_tof_ground_risk(esp32)       # [L,R] ground risk
        imu_data  = read_imu(esp32)                   # [ax,ay,az,gx,gy,gz]
        gps_data  = esp32.gps if esp32 is not None else None

        if esp32 is not None:
            s = esp32.tof_seq
            if s != prev_tof_seq:
                hz_tof.tick()
                prev_tof_seq = s
            s = esp32.imu_seq
            if s != prev_imu_seq:
                hz_imu.tick()
                prev_imu_seq = s
            s = esp32.gps_seq
            if s != prev_gps_seq:
                hz_gps.tick()
                prev_gps_seq = s

        # Sensor fusion (CV + LiDAR inner zones)
        depth_normed    = normalize_depth(depth_map)
        midas_risks_vec = midas_sector_risks(depth_normed)
        yolo_obstacles  = scored_yolo_obstacles(yolo_results, depth_normed, FRAME_WIDTH)
        raw_risks       = fuse_sector_risks(midas_risks_vec, yolo_obstacles,
                                            lidar_sectors=lidar_inner, tof_sectors=tof_risks)
        risks    = smoother.update(raw_risks)

        # Print LiDAR L/C/R motor values to terminal (0-100, slew-limited)
        motor_vals = motor_limiter.limit([r * 100.0 for r in risks])
        print(f"Motor L={motor_vals[0]:3d}  C={motor_vals[1]:3d}  R={motor_vals[2]:3d}", flush=True)

        # Decision with ToF ground + LiDAR flank validation
        decision = fsm.update(risks, lidar_flanks=lidar_flanks, tof_risks=tof_risks)

        # Speak navigation command on change
        if decision.command != last_spoken_command:
            threat = _top_threat_phrase(yolo_obstacles, yolo_model.names)
            if threat:
                speaker.say(f"{threat}. {decision.command}")
            else:
                speaker.say(decision.command)
            last_spoken_command = decision.command

        # Build 5-sector view for telemetry
        fl = lidar_flanks[0] if lidar_flanks else 0.0
        fr = lidar_flanks[1] if lidar_flanks else 0.0
        risks_5 = [fl, risks[0], risks[1], risks[2], fr]

        # Visualization
        if args.viz:
            vis = frame.copy()
            draw_yolo_boxes(vis, yolo_results, yolo_obstacles, depth_normed=depth_normed)
            _draw_lidar_status(vis, lidar_5, hz_lidar.hz)
            _draw_esp32_status(vis, tof_risks, imu_data, gps_data,
                               hz_tof.hz, hz_imu.hz, hz_gps.hz)
            _draw_pipeline_status(vis, hz_cam.hz, hz_yolo.hz, hz_midas.hz)

            # FPS
            t_now = time.time()
            dt    = t_now - t_prev
            t_prev = t_now
            fps_buf.append(1.0 / dt if dt > 0 else 0)
            avg_fps = sum(fps_buf) / len(fps_buf)

            telemetry = build_telemetry_panel(FRAME_WIDTH, decision, risks_5,
                                              fsm.state, avg_fps)
            display = np.vstack([vis, telemetry])

            cv2.imshow("Pathfinder - Integrated", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

    # Cleanup
    stop_event.set()
    t_cap.join(timeout=1.0)
    t_inf.join(timeout=1.0)
    cap.release()
    if args.viz:
        cv2.destroyAllWindows()
    stop_lidar()
    if esp32 is not None:
        esp32.stop()
    print("[integrated] Stopped.")


if __name__ == "__main__":
    main()
