"""Sensor functions for LiDAR, ToF (via ESP32), and IMU.

LiDAR is managed as a persistent instance — call init_lidar() once at startup.
ESP32Reader runs a background UART thread to receive ToF / IMU / GPS packets.
"""

import math
import time
import threading

import numpy as np

from indoor_nav.config import (
    TOF_NOISE_FLOOR_MM, TOF_DEVIATION_SAFE_MM, TOF_DEVIATION_DANGER_MM,
    ESP32_PORT, ESP32_BAUD,
    CALIBRATION_SECS, CALIBRATION_POLL_HZ,
    CALIBRATION_PITCH_TOLERANCE, CALIBRATION_ROLL_TOLERANCE,
)

try:
    import serial as _serial
    _serial_available = True
except ImportError:
    _serial_available = False

try:
    from lidar.api import Lidar
    from haptic_zones import get_haptic_zones
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
    """Read LiDAR sectors [L, C, R] as 0.0-1.0 risk values.

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
        h = get_haptic_zones(data)
        return [h['left'] / 100.0, h['center'] / 100.0, h['right'] / 100.0]
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
        print(f"[ESP32 -> Pi] {msg}")
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
_TOF_LEFT_IDX  = [2, 3, 6, 7, 10, 11, 14, 15]           # columns 0-1
_TOF_RIGHT_IDX = [0, 1, 4, 5, 8, 9, 12, 13]          # columns 2-3


def tof_grid_to_sectors(tof_16):
    """Convert a 16-value ToF grid to [L, R] minimum distances (mm).

    Values below TOF_NOISE_FLOOR_MM are treated as sensor noise and ignored.
    """
    if tof_16 is None or len(tof_16) != 16:
        return None

    def _sector_min(indices):
        valid = [tof_16[i] for i in indices if tof_16[i] >= TOF_NOISE_FLOOR_MM]
        return min(valid) if valid else None

    return [_sector_min(_TOF_LEFT_IDX),
            _sector_min(_TOF_RIGHT_IDX)]


def tof_deviation_to_risk(dist_mm, baseline_mm):
    """Risk based on absolute deviation from calibrated baseline height."""
    if dist_mm is None or baseline_mm is None:
        return 0.0
    deviation = abs(dist_mm - baseline_mm)
    if deviation <= TOF_DEVIATION_SAFE_MM:
        return 0.0
    if deviation >= TOF_DEVIATION_DANGER_MM:
        return 1.0
    return float((deviation - TOF_DEVIATION_SAFE_MM) /
                 (TOF_DEVIATION_DANGER_MM - TOF_DEVIATION_SAFE_MM))


def read_tof_ground_risk(esp32, tof_baseline=None):
    """Get ground-level [L, R] risk scores from ESP32 ToF, or None."""
    if esp32 is None:
        return None
    sectors = tof_grid_to_sectors(esp32.tof)
    if sectors is None:
        return None
    if tof_baseline is None:
        return [0.0, 0.0]
    return [tof_deviation_to_risk(d, b) for d, b in zip(sectors, tof_baseline)]


def read_imu(esp32):
    """Return latest IMU values from ESP32, or None."""
    if esp32 is None:
        return None
    return esp32.imu


# ── Calibration & Posture ─────────────────────────────────────────────

def _accel_to_pitch_roll(ax, ay, az):
    """Extract pitch and roll (degrees) from IMU data.

    Note: The ESP32 IMU returns pitch/roll directly (not raw accelerometer values).
    IMU[0] = pitch (degrees), IMU[1] = roll (degrees), IMU[2] = Z (not used).
    """
    return float(ax), float(ay)


def run_calibration(esp32, duration_secs=CALIBRATION_SECS):
    """Collect ToF + IMU samples over *duration_secs* while user stands still.

    Returns:
        (tof_baseline, baseline_pitch, baseline_roll) on success, or None.
        tof_baseline is [L_mm, R_mm] average ground distances.
    """
    if esp32 is None:
        return None

    tof_l, tof_r = [], []
    pitches, rolls = [], []
    poll_interval = 1.0 / CALIBRATION_POLL_HZ
    end_time = time.time() + duration_secs

    while time.time() < end_time:
        tof = esp32.tof
        if tof is not None:
            sectors = tof_grid_to_sectors(tof)
            if sectors is not None:
                if sectors[0] is not None:
                    tof_l.append(sectors[0])
                if sectors[1] is not None:
                    tof_r.append(sectors[1])

        imu = esp32.imu
        if imu is not None and len(imu) >= 3:
            p, r = _accel_to_pitch_roll(imu[0], imu[1], imu[2])
            pitches.append(p)
            rolls.append(r)

        time.sleep(poll_interval)

    if not tof_l or not tof_r or not pitches:
        return None

    tof_baseline = [sum(tof_l) / len(tof_l), sum(tof_r) / len(tof_r)]
    baseline_pitch = sum(pitches) / len(pitches)
    baseline_roll  = sum(rolls) / len(rolls)

    print(f"[calibration] ToF baseline: L={tof_baseline[0]:.0f}mm R={tof_baseline[1]:.0f}mm "
          f"({len(tof_l)} samples)")
    print(f"[calibration] IMU baseline: pitch={baseline_pitch:+.1f}° roll={baseline_roll:+.1f}° "
          f"({len(pitches)} samples)")
    # Debug: show first IMU reading to check units
    if esp32 is not None and esp32.imu is not None:
        imu_raw = esp32.imu
        p_test, r_test = _accel_to_pitch_roll(imu_raw[0], imu_raw[1], imu_raw[2])
        print(f"[calibration] DEBUG - Raw IMU[0:3]={imu_raw[0]:.1f}, {imu_raw[1]:.1f}, {imu_raw[2]:.1f} => pitch={p_test:.1f}° roll={r_test:.1f}°")

    return tof_baseline, baseline_pitch, baseline_roll


class PostureMonitor:
    """Monitors posture deviation from a pre-calibrated IMU baseline."""

    SLOUCH_GRACE_SECS = 0.5

    def __init__(self, baseline_pitch, baseline_roll,
                 pitch_tol=CALIBRATION_PITCH_TOLERANCE,
                 roll_tol=CALIBRATION_ROLL_TOLERANCE):
        self.baseline_pitch = baseline_pitch
        self.baseline_roll = baseline_roll
        self._pitch_tol = pitch_tol
        self._roll_tol = roll_tol
        self._posture_ok = True
        self._slouch_start = None

    @property
    def posture_ok(self):
        return self._posture_ok

    @property
    def tof_suppressed(self):
        if self._posture_ok or self._slouch_start is None:
            return False
        return (time.time() - self._slouch_start) >= self.SLOUCH_GRACE_SECS

    def update(self, imu_data):
        """Feed IMU reading. Returns True if posture state changed."""
        if imu_data is None or len(imu_data) < 3:
            return False

        pitch, roll = _accel_to_pitch_roll(imu_data[0], imu_data[1], imu_data[2])

        delta_pitch = abs(pitch - self.baseline_pitch)
        delta_roll = abs(roll - self.baseline_roll)

        was_ok = self._posture_ok
        self._posture_ok = (delta_pitch <= self._pitch_tol and
                            delta_roll <= self._roll_tol)

        if self._posture_ok:
            self._slouch_start = None
        elif self._slouch_start is None:
            self._slouch_start = time.time()

        return self._posture_ok != was_ok


