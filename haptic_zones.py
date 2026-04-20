"""
Copied from lidar/api.py with additional haptic feedback implementation.
This file is isolated for easy copying to other branches.
"""

import time

from indoor_nav.config import (
    DIST_FAR_CENTER_M, DIST_NEAR_CENTER_M,
    DIST_FAR_SIDE_M, DIST_NEAR_SIDE_M
)


# ── Slew-rate limiter ──────────────────────────────────────────────────

# Maximum change in motor value (0-100) allowed per second.
# At ~10 Hz updates this is ~25 per tick → 0→100 in ~0.4 s (responsive
# but no instantaneous current spike).
MAX_MOTOR_SLEW_RATE = 300          # units per second
NUM_ZONES   = 3
MAX_ACTIVE_MOTORS = 2

#  HAPTIC MOTOR OUTPUT LIMITS
# Raw vibration intent is produced in 0-100, then remapped before UART output.
# 1) [0, MOTOR_INTENSITY_DEADZONE] -> 0 (deadzone)
# 2) (MOTOR_INTENSITY_DEADZONE, 100] -> [MIN_MOTOR_INTENSITY, MAX_MOTOR_INTENSITY]
MOTOR_INTENSITY_DEADZONE = 30.0
MIN_MOTOR_INTENSITY = 5.0
MAX_MOTOR_INTENSITY = 40.0


class HapticOutputLimiter:
    """Rate-limits and remaps motor values to protect hardware.

    Each call to ``limit()`` returns adjusted motor values whose per-step
    change is bounded by *MAX_MOTOR_SLEW_RATE × dt* so that even if the
    raw input jumps from 0 → 100 the actual output ramps up smoothly.

    Input intent values are first clamped to 0-100 and remapped to motor-safe
    output using:
    - [0, deadzone] -> 0
    - (deadzone, 100] -> [min_intensity, max_intensity] linearly

    The limiter is **time-based** so it adapts to varying loop rates.
    """

    def __init__(self, num_zones: int = NUM_ZONES,
                 max_slew: float = MAX_MOTOR_SLEW_RATE,
                 max_active_motors: int = MAX_ACTIVE_MOTORS,
                 deadzone: float = MOTOR_INTENSITY_DEADZONE,
                 min_intensity: float = MIN_MOTOR_INTENSITY,
                 max_intensity: float = MAX_MOTOR_INTENSITY):
        self._prev = [0.0] * num_zones
        self._last_t = None
        self._max_slew = max_slew          # units / second
        self._max_active_motors = max(0, min(num_zones, int(max_active_motors)))
        self._deadzone = max(0.0, min(100.0, float(deadzone)))
        self._min_intensity = max(0.0, min(100.0, float(min_intensity)))
        self._max_intensity = max(0.0, min(100.0, float(max_intensity)))
        if self._min_intensity > self._max_intensity:
            self._min_intensity, self._max_intensity = (
                self._max_intensity,
                self._min_intensity,
            )

    def _keep_strongest_targets(self, raw: list[float]) -> list[float]:
        """Keep only the strongest requested motors active for this update."""
        if self._max_active_motors >= len(raw):
            return [max(0.0, min(100.0, float(value))) for value in raw]

        clamped = [max(0.0, min(100.0, float(value))) for value in raw]
        ranked = sorted(enumerate(clamped), key=lambda item: item[1], reverse=True)
        keep = {
            index
            for index, value in ranked[:self._max_active_motors]
            if value > 0.0
        }
        return [value if index in keep else 0.0 for index, value in enumerate(clamped)]

    def _remap_intensity(self, value_0_100: float) -> float:
        """Map 0-100 intent to deadzone + [min,max] output range."""
        v = max(0.0, min(100.0, value_0_100))

        if v <= self._deadzone:
            return 0.0

        if self._deadzone >= 100.0:
            return self._max_intensity

        span_in = 100.0 - self._deadzone
        t = (v - self._deadzone) / span_in
        return self._min_intensity + t * (self._max_intensity - self._min_intensity)

    def limit(self, raw: list[float]) -> list[int]:
        """Apply slew-rate limiting and return safe integer motor values.

        Args:
            raw: desired motor values in 0-100 range (one per zone).

        Returns:
            list[int]: strongest-only, deadzoned, remapped, and slew-limited motor values.
        """
        raw = self._keep_strongest_targets(raw)
        now = time.monotonic()
        if self._last_t is None:
            # First call — accept values directly (no spike on boot)
            dt = 0.0
        else:
            dt = now - self._last_t
        self._last_t = now

        max_step = self._max_slew * dt      # allowed change this tick

        out = []
        for i, target in enumerate(raw):
            target = self._remap_intensity(float(target))
            prev = self._prev[i]
            delta = target - prev
            if target <= 0.0:
                target = 0.0
            elif abs(delta) > max_step and max_step > 0:
                target = prev + max_step * (1 if delta > 0 else -1)
            target = max(0.0, min(100.0, target))
            out.append(int(round(target)))
            self._prev[i] = target

        return out


def get_haptic_zones(lidar_data):
    """
    Extract haptic vibration values from LIDAR data for a wearable device.
    
    Divides the front field of view into 3 zones (left, center, right)
    and returns normalized vibration intensities for each.
    Center and side zones use separate distance thresholds from config.
    
    Args:
        lidar_data (LidarData): LIDAR scan data object
    
    Returns:
        dict: {
            'left': vibration intensity (0-100),
            'center': vibration intensity (0-100),
            'right': vibration intensity (0-100),
            'left_distance': closest distance in left zone (mm) or None,
            'center_distance': closest distance in center zone (mm) or None,
            'right_distance': closest distance in right zone (mm) or None
        }
    
    Zone layout (assuming forward = 0 degrees):
        - Left: 290-345 degrees (55 degree span)
        - Center: 345-15 degrees (30 degree span, wraps around 0)
        - Right: 15-70 degrees (55 degree span)
    
    Vibration formula (quadratic ramp):
        x = clamp((max_distance - distance) / (max_distance - full_blast_threshold), 0, 1)
        vibration = x^2 * 100
    """
    
    def normalize_angle(angle):
        """Normalize angle to 0-360 range."""
        return angle % 360.0
    
    def angle_in_range(angle, start, end):
        """Check if angle is in range, accounting for wraparound."""
        angle = normalize_angle(angle)
        start = normalize_angle(start)
        end = normalize_angle(end)
        
        if start <= end:
            return start <= angle <= end
        else:  # Range wraps around 360
            return angle >= start or angle <= end
    
    def distance_to_vibration(distance_mm, max_distance_m, threshold_m):
        """Convert distance in mm to vibration intensity (0-100) using quadratic ramp."""
        if distance_mm is None:
            return 0.0
        
        distance_m = distance_mm / 1000.0
        
        # Clamp normalized distance to [0, 1]
        x = (max_distance_m - distance_m) / (max_distance_m - threshold_m)
        x = max(0.0, min(1.0, x))
        
        # Apply quadratic function, scale to 0-100
        vibration = (x ** 2) * 100.0
        return vibration
    
    # Per-zone distance thresholds (centre vs side)
    zone_thresholds = {
        'left':   (DIST_FAR_SIDE_M,   DIST_NEAR_SIDE_M),
        'center': (DIST_FAR_CENTER_M, DIST_NEAR_CENTER_M),
        'right':  (DIST_FAR_SIDE_M,   DIST_NEAR_SIDE_M),
    }

    # Define the three zones
    zones = {
        'left': (290, 345),      # Left zone: 290-345 degrees
        'center': (345, 15),     # Center zone: 345-15 (wraps around)
        'right': (15, 70)        # Right zone: 15-70 degrees
    }
    
    result = {}
    
    for zone_name, (start, end) in zones.items():
        # Find closest distance in this zone
        closest_distance = None
        for angle, (distance, quality) in lidar_data.items():
            if angle_in_range(angle, start, end) and distance is not None:
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
        
        # Convert to vibration intensity
        far_m, near_m = zone_thresholds[zone_name]
        vibration = distance_to_vibration(closest_distance, far_m, near_m)
        result[zone_name] = vibration
        result[f'{zone_name}_distance'] = closest_distance
    
    return result
