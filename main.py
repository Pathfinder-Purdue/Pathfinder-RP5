import multiprocessing as mp
import time
from utils.latest_queue import LatestQueue

# Our modules
import computer_vision as cv
import esp32_communication as esp32
import motor_strengths as ms


# dummy speed values for testing
LIDAR_HZ = 100
CV_HZ = 5
ESP32_RECV_HZ = 15
ESP32_SEND_HZ = 15
FORCED_MIN_FUSION_HZ = 10  # if we don't get data from all sensors at least this fast, we'll calculate motor strengths with whatever data we have to avoid waiting


def read_lidar(lidar_data: LatestQueue):
    """
    Reads LiDAR data and puts it in the queue.
    """
    # connect to LiDAR and begin reading
    i = 0
    while True:
        # read LiDAR data
        lidar_datum = f"LiDAR data {i}"
        lidar_data.put(lidar_datum)
        i += 1
        time.sleep(1 / LIDAR_HZ)


def read_cv(cv_data: LatestQueue):
    """
    Reads computer vision data and puts it in the queue.
    """
    i = 0
    while True:
        # read CV data
        cv_datum = f"CV data {i}"
        cv_data.put(cv_datum)
        i += 1
        time.sleep(1 / CV_HZ)


def read_esp32(imu_data: LatestQueue, gps_data: LatestQueue):
    """
    Reads ESP32 data and puts it in the queue.
    """
    i = 0
    while True:
        # read ESP32 data
        imu_datum = f"IMU data {i}"
        gps_datum = f"GPS data {i}"
        imu_data.put(imu_datum)
        gps_data.put(gps_datum)
        i += 1
        time.sleep(1 / ESP32_RECV_HZ)


def send_esp32(esp32_outgoing_data: LatestQueue):
    """
    Sends data to the ESP32.
    """
    i = 0
    while True:
        # send data to ESP32
        if not esp32_outgoing_data.empty():
            data_to_send = esp32_outgoing_data.get()
            print(f"Sending to ESP32 [{i}]: {data_to_send}", flush=True)
            i += 1
        time.sleep(1 / ESP32_SEND_HZ)


def calculate_motor_strengths(
    lidar_data: LatestQueue,
    cv_data: LatestQueue,
    imu_data: LatestQueue,
    gps_data: LatestQueue,
    motor_strengths: LatestQueue,
):
    """
    Calculates motor strengths based on LiDAR and CV data and puts it in the queue.
    """
    watchdog_timer = time.time()
    watchdog_timeout = 1 / FORCED_MIN_FUSION_HZ  # seconds

    i = 0
    while True:
        # check if watchdog timer has expired
        if time.time() - watchdog_timer > watchdog_timeout:
            lidar_datum = lidar_data.get() if not lidar_data.empty() else None
            cv_datum = cv_data.get() if not cv_data.empty() else None
            imu_datum = imu_data.get() if not imu_data.empty() else None
            gps_datum = gps_data.get() if not gps_data.empty() else None
            motor_strength = f"Motor strength [{i}] based on {lidar_datum}, {cv_datum}, {imu_datum} (WATCHDOG FORCED)"
            motor_strengths.put(motor_strength)
            i += 1
            watchdog_timer = time.time()  # reset watchdog timer

        # check if we have data from all sensors
        if (
            not lidar_data.empty()
            and not cv_data.empty()
            and not imu_data.empty()
            and not gps_data.empty()
        ):
            lidar_datum = lidar_data.get()
            cv_datum = cv_data.get()
            imu_datum = imu_data.get()
            gps_datum = gps_data.get()
            # calculate motor strengths based on lidar_datum and cv_datum
            motor_strength = (
                f"Motor strength [{i}] based on {lidar_datum}, {cv_datum}, {imu_datum}"
            )
            motor_strengths.put(motor_strength)
            i += 1
            watchdog_timer = time.time()  # reset watchdog timer


def dummy_output_to_terminal(motor_strengths: LatestQueue):
    """
    Dummy function to output motor strengths to terminal.
    """
    while True:
        if not motor_strengths.empty():
            motor_strength = motor_strengths.get()
            print(f"> {motor_strength}", flush=True)


if __name__ == "__main__":
    lidar_data = LatestQueue()
    cv_data = LatestQueue()
    imu_data = LatestQueue()
    gps_data = LatestQueue()
    esp32_outgoing_data = LatestQueue()
    motor_strengths = LatestQueue()

    processes = [
        mp.Process(target=read_lidar, args=(lidar_data,)),
        mp.Process(target=read_cv, args=(cv_data,)),
        mp.Process(
            target=read_esp32,
            args=(
                imu_data,
                gps_data,
            ),
        ),
        mp.Process(target=send_esp32, args=(esp32_outgoing_data,)),
        mp.Process(
            target=calculate_motor_strengths,
            args=(lidar_data, cv_data, imu_data, gps_data, motor_strengths),
        ),
        mp.Process(target=dummy_output_to_terminal, args=(motor_strengths,)),
    ]

    # Theoretical order:
    # 1. read_lidar, read_cv, and read_esp32 run in parallel and fill their respective queues
    # 2. calculate_motor_strengths waits until it has data from all three queues, then calculates motor strengths and puts it in the motor_strengths queue
    # 3. send_esp32 waits until it has data in the esp32_outgoing_data queue, then sends it to the ESP32
    for process in processes:
        process.start()

    print("All processes started.", flush=True)
    print(
        f"LiDAR updates at {LIDAR_HZ} Hz, CV updates at {CV_HZ} Hz, ESP32 recv at {ESP32_RECV_HZ} Hz, ESP32 send at {ESP32_SEND_HZ} Hz, forced fusion at {FORCED_MIN_FUSION_HZ} Hz.",
        flush=True,
    )

    start = time.time()
    while time.time() - start < 10:  # run for 10 seconds then exit
        print("Uptime:", time.time() - start, flush=True)
        while not motor_strengths.empty():
            motor_strength = motor_strengths.get()
            print(f"Calculated motor strength: {motor_strength}", flush=True)
            esp32_outgoing_data.put(motor_strength)
        time.sleep(1)

    for process in processes:
        process.terminate()

    print("All processes terminated.", flush=True)
