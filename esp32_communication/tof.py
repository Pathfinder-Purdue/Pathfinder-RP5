import time
from utils.latest_queue import LatestQueue


TOF_CALIBRATION_DURATION = 1  # seconds to gather calibration data for
TOF_DIMENSION = 4  # ToF data is a 4x4 grid


class TofDataRaw:
    def __init__(self, distance: list[list[float]]):
        self.distance = distance


class TofData:
    def __init__(self, distance: list[list[float]]):
        self.distance = distance

    def calibrate(self, calibration: "TofData") -> None:
        """Calibrates the ToF data by subtracting the calibration values."""
        for i in range(len(self.distance)):
            for j in range(len(self.distance[i])):
                self.distance[i][j] -= calibration.distance[i][j]


class TofDevice:
    def __init__(
        self,
        tof_data: LatestQueue,
        calibration: TofData = TofData([[0.0] * TOF_DIMENSION] * TOF_DIMENSION),
    ):
        self.tof_data = tof_data
        self.calibration = calibration

    def get_calibrated_data(self) -> TofData:
        """Gets the latest ToF data from the queue, calibrates it, and returns it."""
        if not self.tof_data.empty():
            raw_data = self.tof_data.get()  # get the latest raw data from the queue
            clean_data = TofData(raw_data.distance)  # convert raw data to cleaned data
            clean_data.calibrate(
                self.calibration
            )  # calibrate the cleaned data using the provided calibration
            return clean_data
        else:
            return TofData([[0.0] * 8] * 8)  # default data if no data is available


def _clean_tof_data(raw_data: TofDataRaw) -> TofData:
    """Cleans the raw ToF data by converting it to a 2D list of floats."""
    data = [[0.0] * TOF_DIMENSION] * TOF_DIMENSION
    for i in range(TOF_DIMENSION):
        for j in range(TOF_DIMENSION):
            data[i][j] = raw_data.distance[i][j]
    return TofData(raw_data.distance)


def _gather_calibration_data(tof_data: LatestQueue) -> TofData:
    """Gathers calibration data for the ToF sensor by averaging the data over a period of time."""
    start_time = time.time()
    calibration_data = [[0.0] * TOF_DIMENSION] * TOF_DIMENSION
    count = 0

    while time.time() - start_time < TOF_CALIBRATION_DURATION:
        if not tof_data.empty():
            raw_data = tof_data.get()
            clean_data = _clean_tof_data(raw_data)
            for i in range(TOF_DIMENSION):
                for j in range(TOF_DIMENSION):
                    calibration_data[i][j] += clean_data.distance[i][j]
            count += 1

    if count == 0:
        return TofData(
            [[0.0] * TOF_DIMENSION] * TOF_DIMENSION
        )  # default data if no data was gathered

    # Average the calibration data
    for i in range(TOF_DIMENSION):
        for j in range(TOF_DIMENSION):
            calibration_data[i][j] /= count
    return TofData(calibration_data)
