import time
from utils.latest_queue import LatestQueue


class GpsData:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        
class GpsDataRaw:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

def clean_gps_data(raw_data: GpsDataRaw) -> GpsData:
    latitude = raw_data.latitude
    longitude = raw_data.longitude
    altitude = raw_data.altitude
    return GpsData(latitude, longitude, altitude)


def dummy_read_gps_data(gps_data: LatestQueue):
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0
    while True:
        latitude += 0.0001
        longitude += 0.0001
        altitude += 1.0
        
        if latitude > 90.0:
            latitude = -90.0
        if longitude > 180.0:
            longitude = -180.0
        
        gps_data.put(GpsData(latitude, longitude, altitude))
        time.sleep(1)  # Adjust the sleep time as needed