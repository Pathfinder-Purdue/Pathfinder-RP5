"""Sensor functions for LiDAR, ToF (via ESP32), and IMU.

LiDAR is managed as a persistent instance — call init_lidar() once at startup.
ESP32Reader runs a background UART thread to receive ToF / IMU / GPS packets.
"""

import time
import threading

import numpy as np

from indoor_nav.config import (
    DIST_NEAR_M, DIST_FAR_M,
    TOF_NOISE_FLOOR_MM, TOF_CLOSE_MM, TOF_FAR_MM,
    ESP32_PORT, ESP32_BAUD,
)

try:
    import serial as _serial
    _serial_available = True
except ImportError:
    _serial_available = False

try:
    from lidar.api import Lidar
    from haptic_zones import get_haptic_zones_5
    _lidar_available = True
except (ImportError, ModuleNotFoundError):
    _lidar_available = False


# Persistent LiDAR

_lidar_instance = None
_lidar_lock = threading.Lock()


def init_lidar():
    """Initialize the LiDAR once.  Call at startup, not per-frame."""
    global _lidar_instance
    if not _lidar_available:
        print("[sensors] LiDAR modules not available")
        return False
    with _lidar_lock:
        if _lidar_instance is not None:
            return True
        try:
            _lidar_instance = Lidar()
            _lidar_instance.start()
            time.sleep(1.0)  # let the motor spin up
            print("[sensors] LiDAR initialized")
            return True
        except Exception as e:
            print(f"[sensors] LiDAR init failed: {e}")
            _lidar_instance = None
            return False


def stop_lidar():
    """Stop the persistent LiDAR instance."""
    global _lidar_instance
    with _lidar_lock:
        if _lidar_instance is not None:
            try:
                _lidar_instance.stop()
            except Exception:
                pass
            _lidar_instance = None
            print("[sensors] LiDAR stopped")


def read_lidar_sectors():
    """Read LiDAR sectors [FL, L, C, R, FR] as 0.0-1.0 risk values.

    Uses the persistent instance -- call init_lidar() first.
    Haptic zone values are returned on a 0-100 scale; this function
    normalises them back to 0.0-1.0 for the risk engine.
    """
    if _lidar_instance is None:
        return None
    try:
        data = _lidar_instance.read()
        if len(data) == 0:
            return None
        h = get_haptic_zones_5(data)
        return [h['far_left'] / 100.0, h['left'] / 100.0, h['center'] / 100.0,
                h['right'] / 100.0, h['far_right'] / 100.0]
    except Exception:
        return None


# ── ESP32 UART Reader ─────────────────────────────────────────────────

class ESP32Reader:
    """Background thread that reads and parses ESP32 UART messages.

    Expected wire format (semicolon-separated, newline-terminated):
        Inc: N; TOF: v0,v1,...,v15; IMU: ax,ay,az,gx,gy,gz; GPS: v0,v1,v2\\n
    """

    def __init__(self, port=ESP32_PORT, baud=ESP32_BAUD):
        self.port = port
        self.baud = baud
        self._ser = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._tof = None   # list[int]  — 16 values, mm
        self._imu = None   # list[float] — 6 values
        self._gps = None   # list[float] — 3 values
        self._tof_seq = 0
        self._imu_seq = 0
        self._gps_seq = 0

    def start(self):
        """Open the serial port and launch the reader thread."""
        if not _serial_available:
            print("[sensors] pyserial not installed — ESP32 reader disabled")
            return False
        try:
            self._ser = _serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.2,
                write_timeout=0.2,
            )
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._reader_loop, name="esp32-reader", daemon=True
            )
            self._thread.start()
            print(f"[sensors] ESP32 reader started on {self.port}")
            return True
        except _serial.SerialException as e:
            print(f"[sensors] ESP32 serial open failed: {e}")
            return False

    def stop(self):
        """Stop the reader thread and close the serial port."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        print("[sensors] ESP32 reader stopped")

    # ── background loop ──

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                if self._ser.in_waiting > 0:
                    raw = self._ser.read_until(b'\n')
                    if raw:
                        self._parse_message(
                            raw.decode('utf-8', errors='replace').strip()
                        )
                else:
                    time.sleep(0.005)
            except _serial.SerialException:
                break

    def _parse_message(self, msg):
        """Parse semicolon-delimited UART message into tof / imu / gps."""
        tof = imu = gps = None
        for part in msg.split(';'):
            part = part.strip()
            if part.startswith('TOF:'):
                try:
                    tof = [int(v.strip()) for v in part[4:].split(',')]
                except (ValueError, IndexError):
                    pass
            elif part.startswith('IMU:'):
                try:
                    imu = [float(v.strip()) for v in part[4:].split(',')]
                except (ValueError, IndexError):
                    pass
            elif part.startswith('GPS:'):
                try:
                    gps = [float(v.strip()) for v in part[4:].split(',')]
                except (ValueError, IndexError):
                    pass

        with self._lock:
            if tof is not None and len(tof) == 16:
                self._tof = tof
                self._tof_seq += 1
            if imu is not None and len(imu) == 6:
                self._imu = imu
                self._imu_seq += 1
            if gps is not None and len(gps) == 3:
                self._gps = gps
                self._gps_seq += 1

    # ── public properties ──

    @property
    def tof(self):
        """Latest 4x4 ToF grid (16 ints, mm) or None."""
        with self._lock:
            return list(self._tof) if self._tof is not None else None

    @property
    def imu(self):
        """Latest IMU [v0, v1, v2, v3, v4, v5] or None."""
        with self._lock:
            return list(self._imu) if self._imu is not None else None

    @property
    def gps(self):
        """Latest GPS [v0, v1, v2] or None."""
        with self._lock:
            return list(self._gps) if self._gps is not None else None

    @property
    def tof_seq(self):
        with self._lock:
            return self._tof_seq

    @property
    def imu_seq(self):
        with self._lock:
            return self._imu_seq

    @property
    def gps_seq(self):
        with self._lock:
            return self._gps_seq

    def write(self, msg):
        """Send a string to the ESP32 (appends newline)."""
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.write((msg + '\n').encode('ascii'))
                self._ser.flush()
            except _serial.SerialException:
                pass


# ToF ground-level processing

# 4x4 grid -> L / R column mapping
_TOF_LEFT_IDX  = [0, 1, 4, 5, 8, 9, 12, 13]           # columns 0-1
_TOF_RIGHT_IDX = [2, 3, 6, 7, 10, 11, 14, 15]          # columns 2-3


def tof_grid_to_sectors(tof_16):
    """Convert a 16-value ToF grid to [L, R] minimum distances (mm).

    Values below TOF_NOISE_FLOOR_MM are treated as sensor noise and ignored.
    """
    if tof_16 is None or len(tof_16) != 16:
        return None

    def _sector_min(indices):
        valid = [tof_16[i] for i in indices if tof_16[i] >= TOF_NOISE_FLOOR_MM]
        return min(valid) if valid else TOF_FAR_MM

    return [_sector_min(_TOF_LEFT_IDX),
            _sector_min(_TOF_RIGHT_IDX)]


def tof_to_risk(dist_mm):
    """Linear ramp: distance (mm) → 0.0–1.0 ground-obstacle risk."""
    if dist_mm >= TOF_FAR_MM:
        return 0.0
    if dist_mm <= TOF_CLOSE_MM:
        return 1.0
    return float((TOF_FAR_MM - dist_mm) / (TOF_FAR_MM - TOF_CLOSE_MM))


def read_tof_ground_risk(esp32):
    """Get ground-level [L, R] risk scores from ESP32 ToF, or None."""
    if esp32 is None:
        return None
    sectors = tof_grid_to_sectors(esp32.tof)
    if sectors is None:
        return None
    return [tof_to_risk(d) for d in sectors]


def read_imu(esp32):
    """Return latest IMU values from ESP32, or None."""
    if esp32 is None:
        return None
    return esp32.imu


# ── Utility ───────────────────────────────────────────────────────────

def distance_to_risk(dist_m, near_m=DIST_NEAR_M, far_m=DIST_FAR_M):
    """Linear ramp from distance in metres to 0-1 risk score."""
    if dist_m is None:
        return 0.0
    return float(np.clip(1.0 - (dist_m - near_m) / (far_m - near_m), 0.0, 1.0))
