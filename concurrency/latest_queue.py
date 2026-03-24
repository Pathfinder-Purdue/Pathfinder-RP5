import multiprocessing as mp
import queue


class LatestQueue:
    """
    A multiprocessing queue that only keeps the latest item.
    If you put an item in the queue while it's full, it will drop the old item and put the new one.
    This is useful for sensor data where we only care about the most recent reading and don't want to back up if the consumer is slow.
    """

    def __init__(self):
        self._q = mp.Queue(maxsize=1)  # only keep the latest item

    def put(self, item):
        """Non-blocking put. Drops old item if full."""
        try:
            self._q.put_nowait(item)
        except queue.Full:  # drop old item and try again
            try:
                self._q.get_nowait()  # discard stale
            except queue.Empty:  # if another process got there first, just ignore
                pass
            self._q.put_nowait(item)

    def get(self, block=True, timeout=None):
        """Standard get (blocking or non-blocking)."""
        return self._q.get(block=block, timeout=timeout)

    def get_latest(self):
        """
        Drain the queue and return the newest item.
        Returns None if no data available.
        """
        latest = None
        while True:
            try:
                latest = self._q.get_nowait()
            except queue.Empty:
                break
        return latest

    def empty(self):
        return self._q.empty()
