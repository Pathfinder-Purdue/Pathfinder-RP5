"""
Copied from lidar/api.py with additional haptic feedback implementation.
This file is isolated for easy copying to other branches.
"""

import threading
import time


# ── Slew-rate limiter ──────────────────────────────────────────────────

# Maximum change in motor value (0-100) allowed per second.
# At ~10 Hz updates this is ~25 per tick → 0→100 in ~0.4 s (responsive
# but no instantaneous current spike).
MAX_MOTOR_SLEW_RATE = 250          # units per second
DEFAULT_NUM_ZONES   = 3


class HapticOutputLimiter:
    """Rate-limits motor value changes to protect hardware.

    Each call to ``limit()`` returns adjusted motor values whose per-step
    change is bounded by *MAX_MOTOR_SLEW_RATE × dt* so that even if the
    raw input jumps from 0 → 100 the actual output ramps up smoothly.

    The limiter is **time-based** so it adapts to varying loop rates.
    """

    def __init__(self, num_zones: int = DEFAULT_NUM_ZONES,
                 max_slew: float = MAX_MOTOR_SLEW_RATE):
        self._prev = [0.0] * num_zones
        self._last_t = None
        self._max_slew = max_slew          # units / second

    def limit(self, raw: list[float]) -> list[int]:
        """Apply slew-rate limiting and return safe integer motor values.

        Args:
            raw: desired motor values in 0-100 range (one per zone).

        Returns:
            list[int]: clamped, slew-limited motor values 0-100.
        """
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
            target = max(0.0, min(100.0, target))   # hard clamp
            prev = self._prev[i]
            delta = target - prev
            if abs(delta) > max_step and max_step > 0:
                target = prev + max_step * (1 if delta > 0 else -1)
            target = max(0.0, min(100.0, target))
            out.append(int(round(target)))
            self._prev[i] = target

        return out


class LidarData:
    """
    Container for LIDAR scan results.
    Dict of all angles (0-360 deg) -> (distance in mm, quality)
    """
    def __init__(self, data):
        self.data = {}
        for angle, (distance, quality) in sorted(data.items()):
            if distance is not None:
                distance = float(distance)
            self.data[float(angle)] = (distance, int(quality))

    def __iter__(self):
        return iter(self.data.items())

    def __getitem__(self, angle):
        return self.data[angle]
    
    def __len__(self):
        return len(self.data)

    def keys(self):
        return self.data.keys()

    def values(self):
        return self.data.values()

    def items(self):
        return self.data.items()


def get_haptic_zones(lidar_data, max_distance=3000.0, full_blast_threshold=1500.0):
    """
    Extract haptic vibration values from LIDAR data for a wearable device.
    
    Divides the 120-degree front field of view into 3 equal 40-degree zones
    (left, center, right) and returns normalized vibration intensities for each.
    
    Args:
        lidar_data (LidarData): LIDAR scan data object
        max_distance (float): Maximum detection distance in mm (default 3000 mm = 3.0 m)
        full_blast_threshold (float): Distance in mm for maximum vibration (default 1500 mm = 1.5 m)
    
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
    
    def distance_to_vibration(distance_mm, max_dist, threshold):
        """Convert distance in mm to vibration intensity (0-100) using quadratic ramp."""
        if distance_mm is None:
            return 0.0
        
        distance_m = distance_mm / 1000.0
        max_dist_m = max_distance / 1000.0
        threshold_m = full_blast_threshold / 1000.0
        
        # Clamp normalized distance to [0, 1]
        x = (max_dist_m - distance_m) / (max_dist_m - threshold_m)
        x = max(0.0, min(1.0, x))
        
        # Apply quadratic function, scale to 0-100
        vibration = (x ** 2) * 100.0
        return vibration
    
    # Define the three 40-degree zones
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
        vibration = distance_to_vibration(closest_distance, max_distance, full_blast_threshold)
        result[zone_name] = vibration
        result[f'{zone_name}_distance'] = closest_distance
    
    return result


def get_haptic_zones_5(lidar_data, max_distance=3000.0, full_blast_threshold=1500.0):
    """Split 120-degree front FOV into 5 zones: left_bottom, left, center, right, right_bottom.

    LB and RB cover the extra ~17.5 degrees on each side beyond the camera FOV.
    Zone layout (forward = 0 degrees):
        left_bottom:  300-320   (20 degrees, LiDAR-only)
        left:         320-340   (20 degrees)
        center:       340-20    (40 degrees, wraps around 0)
        right:        20-40     (20 degrees)
        right_bottom: 40-60     (20 degrees, LiDAR-only)
    """

    def normalize_angle(angle):
        return angle % 360.0

    def angle_in_range(angle, start, end):
        angle = normalize_angle(angle)
        start = normalize_angle(start)
        end = normalize_angle(end)
        if start <= end:
            return start <= angle <= end
        else:
            return angle >= start or angle <= end

    def distance_to_vibration(distance_mm):
        if distance_mm is None:
            return 0.0
        max_m = max_distance / 1000.0
        thresh_m = full_blast_threshold / 1000.0
        x = (max_m - distance_mm / 1000.0) / (max_m - thresh_m)
        x = max(0.0, min(1.0, x))
        return (x ** 2) * 100.0

    zones = {
        'left_bottom':  (300, 320),
        'left':         (320, 340),
        'center':       (340, 20),
        'right':        (20, 40),
        'right_bottom': (40, 60),
    }

    result = {}
    for zone_name, (start, end) in zones.items():
        closest_distance = None
        for angle, (distance, quality) in lidar_data.items():
            if angle_in_range(angle, start, end) and distance is not None:
                if closest_distance is None or distance < closest_distance:
                    closest_distance = distance
        result[zone_name] = distance_to_vibration(closest_distance)
        result[f'{zone_name}_distance'] = closest_distance

    return result
