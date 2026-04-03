from haptic_zones import get_haptic_zones
import time


def haptic_feedback_demo():
    """
    Real-time haptic feedback display for wearable device.
    Shows normalized vibration intensities for left, center, and right zones at 5Hz.
    """
    # Import Lidar from the main repo
    from lidar.api import Lidar
    
    lidar = Lidar()
    lidar.start()
    
    try:
        while True:
            time.sleep(0.2)  # 5 Hz update rate
            
            data = lidar.read()
            haptic_data = get_haptic_zones(data)
            
            left_vib = haptic_data['left']
            center_vib = haptic_data['center']
            right_vib = haptic_data['right']
            
            # Print simple format
            print(f"Left: {left_vib:.2f} | Center: {center_vib:.2f} | Right: {right_vib:.2f}")
            
    except KeyboardInterrupt:
        lidar.stop()
        print("\nHaptic feedback stopped.")


if __name__ == "__main__":
    haptic_feedback_demo()
