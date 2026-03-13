from lidar.rplidarc1.scanner import RPLidar
import threading
import time
import logging
from typing import Dict, Optional, Tuple
from lidar.rplidarc1.protocol import Request, Response, ResponseMode, RequestBytes

_scanner_thread: Optional[threading.Thread] = None
_stop_event = threading.Event()
_latest_lock = threading.Lock()
_latest_reads: Dict[float, Tuple[Optional[float], int]] = {}
_lidar: Optional[RPLidar] = None

logger = logging.getLogger(__name__)


def start_scanner(port: str, baud: int):
    global _scanner_thread, _stop_event, _lidar
    if _scanner_thread and _scanner_thread.is_alive():
        logger.debug("Scanner already running")
        return

    _stop_event.clear()
    last_error = ValueError("Failed to initialize RPLidar after multiple attempts")
    for attempt in range(10):
        try:
            _lidar = RPLidar(port, baud)
            break  # Success
        except Exception as e:
            last_error = e
            logger.warning(f"RPLidar init failed (attempt {attempt+1}/10): {e}")
            time.sleep(0.5)
    else:
        # All attempts failed
        raise last_error

    _scanner_thread = threading.Thread(target=_scanner_worker, args=(), daemon=True)
    _scanner_thread.start()


def stop_scanner(timeout: float = 5.0):
    """Signal the scanner to stop and wait for thread to join."""
    global _scanner_thread, _stop_event, _lidar
    _stop_event.set()
    if _scanner_thread:
        _scanner_thread.join(timeout)
        if _scanner_thread.is_alive():
            logger.warning("Scanner thread did not exit within timeout")
    if _lidar:
        try:
            _lidar.reset()
        except Exception:
            logger.exception("Error resetting lidar")
        _lidar = None


def is_running():
    return _scanner_thread is not None and _scanner_thread.is_alive()


def get_latest_reads():
    """Return a shallow copy of the latest readings (angle -> distance_mm)."""
    with _latest_lock:
        return dict(_latest_reads)


def _scanner_worker():
    """Background thread: sends scan command and continuously reads/parses packets."""
    global _lidar, _stop_event, _latest_reads
    if _lidar is None:
        logger.error("_scanner_worker started without _lidar instance")
        return

    try:
        scan_request = Request.create_request(RequestBytes.SCAN_BYTE)
        Request.send_request(_lidar._serial, scan_request)
        length, mode = Response.parse_response_descriptor(_lidar._serial)
        if mode != ResponseMode.MUTLI_RESPONSE:
            logger.error("Device returned non-multi response mode for scan")
            return

        byte_error_handling = False
        data_out_buffer = bytes()
        logger.info("Background scanner started")

        while not _stop_event.is_set():
            try:
                if _lidar._serial.in_waiting < 5:
                    time.sleep(0.05)
                    continue

                if not byte_error_handling:
                    data_out_buffer = _lidar._serial.read(length)
                elif data_out_buffer:
                    data_out_buffer = data_out_buffer[1:] + _lidar._serial.read(1)

                if not Response._check_byte_alignment(
                    data_out_buffer[0], data_out_buffer[1]
                ):
                    byte_error_handling = True
                    continue
                else:
                    byte_error_handling = False

                parsed = Response._parse_simple_scan_result(data_out_buffer)
                if parsed is None:
                    continue
                quality, angle, distance = parsed
                distance = None if distance == 0 else distance

                with _latest_lock:
                    _latest_reads[angle] = (distance, quality)

            except Exception:
                logger.exception("Error in scanner loop — retrying after short delay")
                time.sleep(0.5)

    finally:
        logger.info("Background scanner exiting, resetting lidar")
        try:
            if _lidar:
                _lidar.reset()
        except Exception:
            logger.exception("Error during lidar reset in scanner worker")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    start_scanner("COM5", 460800)
    try:
        while True:
            time.sleep(1)
            for i, j in sorted(get_latest_reads().items()):
                print(f"{i}: {j[1]}, {j[0]}")
    except KeyboardInterrupt:
        stop_scanner()
