from lidar.api import Lidar
import time

def terminal_lidar():
    """
    Live LIDAR printing to terminal.
    """
    lidar = Lidar()
    lidar.start()
    try:
        while True:
            time.sleep(0.2)
            data = lidar.read()
            for angle, (distance, quality) in data:
                if distance is not None:
                    distance_str = f"{distance:10.2f}"
                else:
                    distance_str = "      None"
                print(f"{angle:6.2f} deg: {distance_str} mm, q {quality}")
            
            try:
                print(f"Summary: # of points: {len(data)}")
                print(f"{data[315][0]:7.0f}   {data[0][0]:7.0f}   {data[45][0]:7.0f}")
                print("\n               ^")
                print(f"{data[270][0]:7.0f}        0    {data[90][0]:7.0f}")
                print("              deg\n")
                print(f"{data[225][0]:7.0f}   {data[180][0]:7.0f}   {data[135][0]:7.0f}")
            except:
                pass
                
    except KeyboardInterrupt:
        lidar.stop()


if __name__ == "__main__":
    terminal_lidar()
