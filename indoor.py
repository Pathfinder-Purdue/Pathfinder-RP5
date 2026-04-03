"""Entry point for indoor navigation mode. Wires together all the modules."""

import sys
import time
from collections import deque
from threading import Event, Lock, Thread

import cv2
import numpy as np
import torch

from indoor_nav.config import (
    CAMERA_ID, FRAME_WIDTH, FRAME_HEIGHT,
    YOLO_INTERVAL, MIDAS_INTERVAL,
)
from indoor_nav.models import load_yolo, load_midas, run_yolo, run_midas
from indoor_nav.sensors import read_lidar_sectors, read_tof_sectors
from indoor_nav.detection import normalize_depth, midas_sector_risks, scored_yolo_obstacles
from indoor_nav.risk_engine import fuse_sector_risks, RiskSmoother
from indoor_nav.decision import NavigationFSM
from indoor_nav.visualization import (
    draw_depth_overlay,
    draw_yolo_boxes,
    build_telemetry_panel,
)


def main():
    # load models
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[indoor] device: {device}")

    print("[indoor] Loading YOLO …")
    yolo_model = load_yolo()
    print("[indoor] Loading MiDaS …")
    midas_model, midas_transform = load_midas(device)
    print("[indoor] Models ready.")

    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        sys.exit("[indoor] ERROR — could not open camera")

    # pipeline components
    fsm = NavigationFSM()
    smoother = RiskSmoother()

    risks = [0.0, 0.0, 0.0]

    fps_buf = deque(maxlen=30)
    t_prev = time.time()

    shared = {
        "latest_frame": None,
        "yolo_results": None,
        "depth_map": None,
        "capture_count": 0,
    }
    data_lock = Lock()
    stop_event = Event()

    def capture_worker():
        """Continuously read frames so capture is not blocked by inference."""
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                stop_event.set()
                return
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            with data_lock:
                shared["latest_frame"] = frame
                shared["capture_count"] += 1

    def inference_worker():
        """Run YOLO and MiDaS on latest frame using staggered scheduling."""
        last_seen_count = -1
        while not stop_event.is_set():
            with data_lock:
                frame = shared["latest_frame"]
                count = shared["capture_count"]

            if frame is None or count == last_seen_count:
                time.sleep(0.001)
                continue

            last_seen_count = count

            yolo_out = None
            depth_out = None

            if count % YOLO_INTERVAL == 0:
                yolo_out = run_yolo(yolo_model, frame)

            if count % MIDAS_INTERVAL == 0:
                depth_out = run_midas(midas_model, midas_transform, frame, device)

            if yolo_out is not None or depth_out is not None:
                with data_lock:
                    if yolo_out is not None:
                        shared["yolo_results"] = yolo_out
                    if depth_out is not None:
                        shared["depth_map"] = depth_out

    t_capture = Thread(target=capture_worker, name="capture-worker", daemon=True)
    t_infer = Thread(target=inference_worker, name="inference-worker", daemon=True)
    t_capture.start()
    t_infer.start()

    print("[indoor] Streaming — press 'q' to quit")

    # frame loop
    while not stop_event.is_set():
        with data_lock:
            frame = None if shared["latest_frame"] is None else shared["latest_frame"].copy()
            yolo_results = shared["yolo_results"]
            depth_map = shared["depth_map"]

        if frame is None:
            time.sleep(0.001)
            continue

        # physical sensors (empty for now)
        lidar = read_lidar_sectors()
        tof   = read_tof_sectors()

        # fuse everything into sector risks
        depth_normed    = normalize_depth(depth_map)
        midas_risks_vec = midas_sector_risks(depth_normed)
        yolo_obstacles  = scored_yolo_obstacles(yolo_results, depth_normed, FRAME_WIDTH)
        raw_risks       = fuse_sector_risks(midas_risks_vec, yolo_obstacles, lidar, tof)
        risks           = smoother.update(raw_risks)

        # get nav command from FSM
        decision = fsm.update(risks)

        # draw everything
        vis = frame.copy()
        draw_depth_overlay(vis, depth_map, alpha=0.25)
        draw_yolo_boxes(vis, yolo_results, yolo_obstacles)

        # fps
        t_now = time.time()
        dt = t_now - t_prev
        t_prev = t_now
        fps_buf.append(1.0 / dt if dt > 0 else 0)
        avg_fps = sum(fps_buf) / len(fps_buf)

        telemetry = build_telemetry_panel(FRAME_WIDTH, decision, risks, fsm.state, avg_fps)
        display = np.vstack([vis, telemetry])

        cv2.imshow("Pathfinder - Indoor Mode", display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            stop_event.set()
            break

    stop_event.set()
    t_capture.join(timeout=1.0)
    t_infer.join(timeout=1.0)

    cap.release()
    cv2.destroyAllWindows()
    print("[indoor] Stopped.")


if __name__ == "__main__":
    main()

