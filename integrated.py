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
    STOP_THRESHOLD,
)
from indoor_nav.models import load_yolo, load_midas, run_yolo, run_midas
from indoor_nav.sensors import (
    ESP32Reader, read_tof_ground_risk, read_imu,
    init_lidar, stop_lidar, read_lidar_sectors,
    PostureMonitor, run_calibration,
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


# Text to speech settings for navigation cues
_SECTOR_NAMES = ["to your left", "ahead", "to your right"]
_DANGER_OBJECT_PROX_THRESHOLD = 0.70
_POSTURE_REPEAT_SECS = 2.0
_DANGER_SPEECH_COOLDOWN_SECS = 1.5


def _top_threat_phrase(yolo_obstacles, class_names, min_proximity=0.3):
    """Return e.g. 'Person ahead' for the closest YOLO obstacle, or None."""
    if not yolo_obstacles:
        return None
    # higher proximity means the object is closer
    sector, prox, box = max(yolo_obstacles, key=lambda o: o[1])
    if prox < min_proximity:
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


# Simple rolling frequency tracker
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


def _draw_lidar_status(frame, lidar_3, hz):
    """Draw LiDAR [L, C, R] risk values in the top-right corner."""
    x = frame.shape[1] - 310
    y = 18
    if lidar_3 is not None:
        peak = max(lidar_3)
        color = _RED if peak > 0.7 else _GREEN
        cv2.putText(frame,
                    f"LiDAR [{hz:.1f}Hz]: L={lidar_3[0]:.0%} C={lidar_3[1]:.0%} R={lidar_3[2]:.0%}",
                    (x, y), _FONT, 0.38, color, 1, cv2.LINE_AA)
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

    # load models
    print("[integrated] Loading YOLO ...")
    yolo_model = load_yolo()
    print("[integrated] Loading MiDaS ...")
    midas_model, midas_transform = load_midas(device)
    print("[integrated] Models ready.")

    # open camera
    print("[integrated] Opening camera ...")
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        sys.exit("[integrated] ERROR -- could not open camera")
    for _ in range(10):
        cap.read()
    print("[integrated] Camera ready.")

    # start LiDAR
    print("[integrated] Starting LiDAR ...")
    lidar_ok = init_lidar()
    if not lidar_ok:
        print("[integrated] WARNING -- LiDAR unavailable")

    # start ESP32 UART reader
    print("[integrated] Starting ESP32 reader ...")
    esp32 = ESP32Reader()
    esp32_ok = esp32.start()
    if not esp32_ok:
        print("[integrated] WARNING -- ESP32 unavailable, ToF/IMU/GPS disabled")
        esp32 = None

    # display variables
    time_since_last_print = time.time()

    # runtime state
    fsm      = NavigationFSM()
    smoother = RiskSmoother()
    risks    = [0.0, 0.0, 0.0]

    lidar_sectors = None
    tof_risks    = None
    imu_data     = None
    gps_data     = None

    # start the TTS worker
    speaker = Speaker()
    speaker.start()
    last_posture_prompt_ts = 0.0
    last_danger_phrase = None
    last_danger_phrase_ts = 0.0

    # wait before starting calibration in headless mode for bootup
    if not args.viz:
        print("[integrated] Waiting 90 seconds before calibration ...")
        speaker.say("System starting, 90 seconds until calibration")
        time.sleep(90.0)

    # calibration period
    print("[integrated] Starting calibration ...")
    speaker.say("Calibration started, hold still and stand upright")
    time.sleep(1.5)  # let speech finish and user settle

    cal_result = run_calibration(esp32)
    if cal_result is not None:
        tof_baseline, baseline_pitch, baseline_roll = cal_result
        posture = PostureMonitor(baseline_pitch, baseline_roll)
    else:
        tof_baseline = None
        posture = None
        print("[integrated] WARNING -- calibration failed (no ESP32 data)")

    speaker.say("Calibration ended, Pathfinder online")
    print("[integrated] Pathfinder online")

    fps_buf = deque(maxlen=30)
    t_prev  = time.time()

    # frequency trackers for each sensor path
    hz_cam   = HzTracker()
    hz_yolo  = HzTracker()
    hz_midas = HzTracker()
    hz_lidar = HzTracker()
    hz_tof   = HzTracker()
    hz_imu   = HzTracker()
    hz_gps   = HzTracker()

    # sequence counters for new ESP32 samples
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

    # camera capture thread
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

    # inference thread (YOLO + MiDaS)
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

    # launch worker threads
    t_cap = Thread(target=capture_worker,  name="capture-worker",  daemon=True)
    t_inf = Thread(target=inference_worker, name="inference-worker", daemon=True)
    t_cap.start()
    t_inf.start()
    print("[integrated] Streaming -- press 'q' to quit")

    motor_limiter = HapticOutputLimiter(num_zones=5)

    # main frame loop
    while not stop_event.is_set():
        # read latest frame and inference outputs
        with data_lock:
            frame = (None if shared["latest_frame"] is None
                     else shared["latest_frame"].copy())
            yolo_results = shared["yolo_results"]
            depth_map    = shared["depth_map"]

        if frame is None:
            time.sleep(0.001)
            continue

        # read LiDAR sector risks
        lidar_sectors = read_lidar_sectors()            # [L,C,R] 0-1 risk
        if lidar_sectors is not None:
            hz_lidar.tick()

        # read ESP32 sensors and update rates on new samples
        tof_risks = read_tof_ground_risk(esp32, tof_baseline)  # [L,R] ground risk
        imu_data  = read_imu(esp32)                   # [ax,ay,az,gx,gy,gz]
        gps_data  = esp32.gps if esp32 is not None else None

        # monitor posture continuously and suppress ToF if user slouches
        if posture is not None:
            posture_changed = posture.update(imu_data)
            now = time.time()
            if not posture.posture_ok and (now - last_posture_prompt_ts) >= _POSTURE_REPEAT_SECS:
                speaker.say("Straighten your posture")
                last_posture_prompt_ts = now
            elif posture_changed and posture.posture_ok:
                speaker.say("Posture restored")

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

        # fuse LiDAR, MiDaS, and YOLO into L/C/R risks
        depth_normed    = normalize_depth(depth_map)
        midas_risks_vec = midas_sector_risks(depth_normed)
        yolo_obstacles  = scored_yolo_obstacles(yolo_results, depth_normed, FRAME_WIDTH)
        raw_risks       = fuse_sector_risks(midas_risks_vec, yolo_obstacles,
                                            lidar_sectors=lidar_sectors)
        risks    = smoother.update(raw_risks)

        # convert ToF to left-bottom and right-bottom ground risks
        tof_lb = tof_risks[0] if tof_risks else 0.0
        tof_rb = tof_risks[1] if tof_risks else 0.0

        # five-zone motor output: [LB, L, C, R, RB]
        motor_raw = [tof_lb * 100.0, risks[0] * 100.0, risks[1] * 100.0,
                     risks[2] * 100.0, tof_rb * 100.0]
        if posture is not None and posture.tof_suppressed:
            # pulse all motors at 3 Hz when posture is bad
            pulse_on = int(time.time() * 3) % 2 == 0
            motor_raw = [100, 100, 100, 100, 100] if pulse_on else [0, 0, 0, 0, 0]
        motor_vals = motor_limiter.limit(motor_raw)

        # send motor values to ESP32
        if esp32 is not None:
            esp32.write_motor_values(motor_vals)

        # navigation decision
        decision = fsm.update(risks)

        # in danger, speak object location only
        danger_zone = max(risks) >= STOP_THRESHOLD
        if danger_zone:
            threat = _top_threat_phrase(
                yolo_obstacles,
                yolo_model.names,
                min_proximity=_DANGER_OBJECT_PROX_THRESHOLD,
            )
            if threat is not None:
                now = time.time()
                should_repeat = (now - last_danger_phrase_ts) >= _DANGER_SPEECH_COOLDOWN_SECS
                if threat != last_danger_phrase or should_repeat:
                    speaker.say(threat)
                    last_danger_phrase = threat
                    last_danger_phrase_ts = now
        else:
            last_danger_phrase = None

        # build five-sector telemetry view: [LB, L, C, R, RB]
        risks_5 = [tof_lb, risks[0], risks[1], risks[2], tof_rb]

        if time.time() - time_since_last_print >= 0.1:
            print(f"[risks] LiDAR: L={risks[0]:.0%} C={risks[1]:.0%} R={risks[2]:.0%} | ToF LB={tof_lb:.0%} RB={tof_rb:.0%}")
            print(f"[motor] LB={motor_vals[0]} L={motor_vals[1]} C={motor_vals[2]} R={motor_vals[3]} RB={motor_vals[4]}")
            time_since_last_print = time.time()

        # visualization
        if args.viz:
            vis = frame.copy()
            draw_yolo_boxes(vis, yolo_results, yolo_obstacles, depth_normed=depth_normed)
            _draw_lidar_status(vis, lidar_sectors, hz_lidar.hz)
            _draw_esp32_status(vis, tof_risks, imu_data, gps_data,
                               hz_tof.hz, hz_imu.hz, hz_gps.hz)
            _draw_pipeline_status(vis, hz_cam.hz, hz_yolo.hz, hz_midas.hz)

            # fps
            t_now = time.time()
            dt    = t_now - t_prev
            t_prev = t_now
            fps_buf.append(1.0 / dt if dt > 0 else 0)
            avg_fps = sum(fps_buf) / len(fps_buf)

            motor_display = [v / 100.0 for v in motor_vals]
            telemetry = build_telemetry_panel(
                FRAME_WIDTH,
                decision,
                risks_5,
                fsm.state,
                avg_fps,
                motor_values=motor_display,
            )
            display = np.vstack([vis, telemetry])

            cv2.imshow("Pathfinder - Integrated", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

    # cleanup
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
