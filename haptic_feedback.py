from haptic_zones import get_haptic_zones, HapticOutputLimiter
import time


def haptic_feedback_demo():
    """Show live haptic values for left, center, and right zones."""
    # import Lidar from the local package
    from lidar.api import Lidar
    
    lidar = Lidar()
    lidar.start()
    limiter = HapticOutputLimiter(num_zones=3)
    
    try:
        while True:
            time.sleep(0.2)  # 5 Hz update rate.
            
            data = lidar.read()
            haptic_data = get_haptic_zones(data)
            
            raw = [haptic_data['left'], haptic_data['center'], haptic_data['right']]
            motor_vals = limiter.limit(raw)
            
            # print a compact text view
            print(f"Left: {motor_vals[0]:3d} | Center: {motor_vals[1]:3d} | Right: {motor_vals[2]:3d}")
            
    except KeyboardInterrupt:
        lidar.stop()
        print("\nHaptic feedback stopped.")


if __name__ == "__main__":
    haptic_feedback_demo()
