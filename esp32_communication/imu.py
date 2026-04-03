import time
from utils.latest_queue import LatestQueue


IMU_CALIBRATION_DURATION = 1  # seconds to gather calibration data for
TOF_VALID_PITCH_RANGE = (
    -20,
    20,
)  # ToF data is only valid when the pitch is within this range
TOF_VALID_ROLL_RANGE = (
    -10,
    10,
)  # ToF data is only valid when the roll is within this range


class ImuDataRaw:
    def __init__(self, acc_x, acc_y, acc_z, pitch, roll, yaw):
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw


class ImuData:
    def __init__(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    def calibrate(self, calibration: "ImuData") -> None:
        """Calibrates the IMU data by subtracting the calibration values."""
        self.pitch -= calibration.pitch
        self.roll -= calibration.roll
        self.yaw -= calibration.yaw


class ImuDevice:
    def __init__(
        self, imu_data: LatestQueue, calibration: ImuData = ImuData(0.0, 0.0, 0.0)
    ):
        self.imu_data = imu_data
        self.calibration = calibration

    def calibrate(self):
        """Calibrates the IMU by gathering calibration data and setting the calibration attribute."""
        self.calibration = _gather_calibration_data(self.imu_data)

    def get_calibrated_data(self) -> ImuData:
        """Gets the latest IMU data from the queue, calibrates it, and returns it."""
        if not self.imu_data.empty():
            raw_data = self.imu_data.get()  # get the latest raw data from the queue
            clean_data = _clean_imu_data(raw_data)  # convert raw data to cleaned data
            clean_data.calibrate(
                self.calibration
            )  # calibrate the cleaned data using the provided calibration
            return clean_data
        else:
            return ImuData(0.0, 0.0, 0.0)  # default data if no data is available

    def is_tof_data_valid(self, datum: ImuData) -> bool:
        """Checks if the ToF data is valid based on the pitch angle."""
        valid = True
        if not (TOF_VALID_PITCH_RANGE[0] <= datum.pitch <= TOF_VALID_PITCH_RANGE[1]):
            valid = False
        if not (TOF_VALID_ROLL_RANGE[0] <= datum.roll <= TOF_VALID_ROLL_RANGE[1]):
            valid = False
        return valid


def _clean_imu_data(raw_data: ImuDataRaw) -> ImuData:
    """Converts raw IMU data to usable IMU data."""
    pitch = raw_data.pitch
    roll = raw_data.roll
    yaw = raw_data.yaw
    return ImuData(pitch, roll, yaw)


def _gather_calibration_data(imu_data: LatestQueue) -> ImuData:
    """Gathers calibration data for the specified duration and returns the average."""
    start_time = time.time()
    pitch_sum = 0.0
    roll_sum = 0.0
    yaw_sum = 0.0
    count = 0

    while time.time() - start_time < IMU_CALIBRATION_DURATION:
        if not imu_data.empty():
            raw_data = imu_data.get()
            clean_data = _clean_imu_data(raw_data)
            pitch_sum += clean_data.pitch
            roll_sum += clean_data.roll
            yaw_sum += clean_data.yaw
            count += 1

    if count == 0:
        return ImuData(0.0, 0.0, 0.0)  # default calibration if no data was gathered

    # Average the calibration data
    return ImuData(pitch_sum / count, roll_sum / count, yaw_sum / count)


def dummy_read_imu_data(imu_data: LatestQueue):
    current_axis = "pitch"
    current_direction = 1
    step = 10.0
    pitch = 0.0
    roll = 0.0
    yaw = 0.0
    for _ in range(20):  # start with some initial data for calibration
        imu_datum = ImuData(pitch, roll, yaw)
        imu_data.put(imu_datum)
        time.sleep(0.1)

    while True:
        if current_axis == "pitch":
            pitch += step * current_direction
            if pitch >= 180.0:
                pitch = 180.0
                current_direction = -1
            elif pitch <= -180.0:
                pitch = -180.0
                current_direction = 1
            if pitch == 0.0 and current_direction == 1:
                current_axis = "roll"
        elif current_axis == "roll":
            roll += step * current_direction
            if roll >= 180.0:
                roll = 180.0
                current_direction = -1
            elif roll <= -180.0:
                roll = -180.0
                current_direction = 1
            if roll == 0.0 and current_direction == 1:
                current_axis = "yaw"
        elif current_axis == "yaw":
            yaw += step * current_direction
            if yaw >= 180.0:
                yaw = 180.0
                current_direction = -1
            elif yaw <= -180.0:
                yaw = -180.0
                current_direction = 1
            if yaw == 0.0 and current_direction == 1:
                current_axis = "pitch"
        imu_datum = ImuData(pitch, roll, yaw)
        imu_data.put(imu_datum)
        time.sleep(0.1)
