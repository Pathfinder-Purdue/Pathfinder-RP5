from haptic_zones import get_haptic_zones, HapticOutputLimiter
import time


def haptic_feedback_demo():
    """
    Real-time haptic feedback display for wearable device.
    Shows slew-limited vibration intensities (0-100) for left, center, and right zones at 5Hz.
    """
    # Import Lidar from the main repo
    from lidar.api import Lidar
    
    lidar = Lidar()
    lidar.start()
    limiter = HapticOutputLimiter(num_zones=3)
    
    try:
        while True:
            time.sleep(0.2)  # 5 Hz update rate
            
            data = lidar.read()
            haptic_data = get_haptic_zones(data)
            
            raw = [haptic_data['left'], haptic_data['center'], haptic_data['right']]
            motor_vals = limiter.limit(raw)
            
            # Print simple format
            print(f"Left: {motor_vals[0]:3d} | Center: {motor_vals[1]:3d} | Right: {motor_vals[2]:3d}")
            
    except KeyboardInterrupt:
        lidar.stop()
        print("\nHaptic feedback stopped.")


if __name__ == "__main__":
    haptic_feedback_demo()
