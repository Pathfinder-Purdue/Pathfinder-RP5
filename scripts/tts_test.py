import subprocess
import time
from queue import Queue, Empty
from threading import Thread


class Speaker:
    """Non-blocking TTS using espeak-ng on a background thread."""
    def __init__(self, rate=160):
        self._queue = Queue(maxsize=1)
        self._rate = rate
        self._thread = None

    def start(self):
        self._thread = Thread(target=self._worker, name="tts-speaker", daemon=True)
        self._thread.start()
        print("[integrated] TTS speaker ready (espeak-ng)")

    def say(self, text):
        """Queue a phrase. Drops old queued phrase if not yet spoken."""
        try:
            self._queue.get_nowait()
        except Empty:
            pass
        self._queue.put(text)

    def _worker(self):
        while True:
            text = self._queue.get()
            try:
                subprocess.run(
                    ["espeak-ng", "-s", str(self._rate), "-a", "200", text],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass


speaker = Speaker()
speaker.start()
speaker.say("sequence initiated. 10. 9. 8. 7. 6. 5. 4. 3. 2. 1. EIEOIEOEIEOEPIPAOIDSOPIASJDOIAPOFBJBDISBWUIAOBWBSAKHBCSJBCIUOAWGPIUEHIWPUHKLJBXLKSJG")
time.sleep(10)  # wait for speech to finish before process exits