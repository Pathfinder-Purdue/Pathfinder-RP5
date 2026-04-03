import esp32_communication.imu as imu
import multiprocessing as mp
import time
import colorama


def main():
    imu_data = imu.LatestQueue()
    imu_process = mp.Process(target=imu.dummy_read_imu_data, args=(imu_data,))
    imu_process.start()
    imu_device = imu.ImuDevice(imu_data)
    print("Process started")
    
    imu_data.get()  # block until we get the first datum
    imu_device.calibrate()
    print(f"Calibration complete: Pitch={imu_device.calibration.pitch}, Roll={imu_device.calibration.roll}, Yaw={imu_device.calibration.yaw}")
    
    while True:
        if not imu_data.empty():
            imu_datum = imu_device.get_calibrated_data()
            valid = imu_device.is_tof_data_valid(imu_datum)
            if valid:
                print(colorama.Fore.GREEN, end="")
            else:
                print(colorama.Fore.RED, end="")
            print(f"IMU Data: Pitch={imu_datum.pitch}, Roll={imu_datum.roll}, Yaw={imu_datum.yaw}, TofValid={valid}")
            print(colorama.Style.RESET_ALL, end="")
        time.sleep(0.001)
        
if __name__ == "__main__":
    main()