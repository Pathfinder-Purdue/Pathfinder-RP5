import time
from utils.latest_queue import LatestQueue


class ImuData:
    def __init__(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.max_angle = 180.0
        self.min_angle = -180.0


class ImuDataRaw:
    def __init__(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.gyro_x = gyro_x
        self.gyro_y = gyro_y
        self.gyro_z = gyro_z

      
def clean_imu_data(raw_data: ImuDataRaw) -> ImuData:
    pitch = raw_data.gyro_x
    roll = raw_data.gyro_y
    yaw = raw_data.gyro_z
    return ImuData(pitch, roll, yaw)


def dummy_read_imu_data(imu_data: LatestQueue):
    current_axis = "pitch"
    current_direction = 1
    step = 10.0
    pitch = 0.0
    roll = 0.0
    yaw = 0.0
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
        