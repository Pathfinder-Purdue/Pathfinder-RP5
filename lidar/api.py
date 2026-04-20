import lidar.backend as backend
import threading
import time
import glob



LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_ac8cbf20826eef118ad1e0c2c169b110-if00-port0"
LIDAR_BAUDRATE = 460800

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

class Lidar:
    """
    Clean API for accessing LIDAR functions from other programs.
    Usage:
        lidar = Lidar()
        lidar.start()
        data = lidar.read()  # returns LidarData
        lidar.stop()
    """
    def __init__(self, port=LIDAR_PORT, baudrate=LIDAR_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self._running = False
        self._lock = threading.Lock()

    def start(self):
        """Start the LIDAR scanner in the background."""
        with self._lock:
            if not self._running:
                backend.start_scanner(self.port, self.baudrate)
                self._running = True

    def read(self):
        """Get the most recent scan data as a LidarData object."""
        return LidarData(backend.get_latest_reads())

    def stop(self):
        """Stop the LIDAR scanner and release resources."""
        with self._lock:
            if self._running:
                backend.stop_scanner()
                self._running = False

# Example usage
if __name__ == "__main__":
    lidar = Lidar()
    lidar.start()
    try:
        while True:
            time.sleep(1)
            data = lidar.read()
            for angle, (distance, quality) in data:
                print("{:.2f}: {}, {}".format(angle, distance, quality))
    except KeyboardInterrupt:
        lidar.stop()
