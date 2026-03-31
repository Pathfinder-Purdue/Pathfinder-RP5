import time
import multiprocessing as mp
import esp32_communication.imu as imu
import esp32_communication.gps as gps


def main():
    imu_data = imu.LatestQueue()
    gps_data = gps.LatestQueue()

    imu_process = mp.Process(target=imu.dummy_read_imu_data, args=(imu_data,))
    gps_process = mp.Process(target=gps.dummy_read_gps_data, args=(gps_data,))

    imu_process.start()
    gps_process.start()

    while True:
        if not imu_data.empty():
            imu_datum = imu_data.get()
            print(f"IMU Data: Pitch={imu_datum.pitch}, Roll={imu_datum.roll}, Yaw={imu_datum.yaw}")
        if not gps_data.empty():
            gps_datum = gps_data.get()
            print(f"GPS Data: Latitude={gps_datum.latitude}, Longitude={gps_datum.longitude}, Altitude={gps_datum.altitude}")
        time.sleep(0.001)
        

if __name__ == "__main__":
    main()