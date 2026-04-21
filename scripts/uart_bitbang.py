"""
Bit-banged UART on Raspberry Pi 5 using gpiod.

Same read/write pattern as uart_test.py but on arbitrary GPIO pins
instead of the hardware UART (/dev/ttyAMA0 on GPIO 14/15).

Default pins: GPIO 17 (TX), GPIO 27 (RX)
Default baud: 9600
"""

import gpiod
import threading
import time

TX_PIN = 17
RX_PIN = 27
BAUD = 9600
BIT_TIME = 1.0 / BAUD  # ~104.17 us at 9600
CHIP = "/dev/gpiochip4"  # Pi 5 main GPIO chip


def _busy_wait_until(target: float):
    """Spin-wait until perf_counter reaches *target*."""
    while time.perf_counter() < target:
        pass


class BitBangUART:
    def __init__(self, chip=CHIP, tx_pin=TX_PIN, rx_pin=RX_PIN, baud=BAUD):
        self.bit_time = 1.0 / baud
        self._chip = gpiod.Chip(chip)

        # TX: output, idle high
        self._tx_req = self._chip.request_lines(
            config={tx_pin: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT)},
            consumer="bitbang-tx",
        )
        self._tx_pin = tx_pin
        self._tx_req.set_value(tx_pin, gpiod.line.Value.ACTIVE)  # idle high

        # RX: input with pull-up (idle high)
        self._rx_req = self._chip.request_lines(
            config={rx_pin: gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT,
                bias=gpiod.line.Bias.PULL_UP,
            )},
            consumer="bitbang-rx",
        )
        self._rx_pin = rx_pin

        self._tx_lock = threading.Lock()

    # TX

    def _write_byte(self, byte: int):
        bt = self.bit_time
        pin = self._tx_pin
        req = self._tx_req

        t = time.perf_counter()

        # start bit (low)
        req.set_value(pin, gpiod.line.Value.INACTIVE)
        t += bt
        _busy_wait_until(t)

        # 8 data bits, LSB first
        for i in range(8):
            if (byte >> i) & 1:
                req.set_value(pin, gpiod.line.Value.ACTIVE)
            else:
                req.set_value(pin, gpiod.line.Value.INACTIVE)
            t += bt
            _busy_wait_until(t)

        # stop bit (high)
        req.set_value(pin, gpiod.line.Value.ACTIVE)
        t += bt
        _busy_wait_until(t)

    def write(self, data: bytes):
        with self._tx_lock:
            for b in data:
                self._write_byte(b)

    # RX

    def _read_byte(self) -> int | None:
        """Block until a start bit is detected, then clock in 8 bits."""
        bt = self.bit_time
        pin = self._rx_pin
        req = self._rx_req

        # Wait for start bit (line goes low)
        while req.get_value(pin) == gpiod.line.Value.ACTIVE:
            pass  # spin on idle-high

        # Sample in the middle of each bit
        t = time.perf_counter() + bt * 1.5  # skip start bit, land mid-bit0

        byte = 0
        for i in range(8):
            _busy_wait_until(t)
            if req.get_value(pin) == gpiod.line.Value.ACTIVE:
                byte |= (1 << i)
            t += bt

        # Wait through the stop bit
        _busy_wait_until(t)
        return byte

    def read_until(self, terminator: bytes = b'\n', timeout: float = 1.0) -> bytes:
        buf = bytearray()
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            b = self._read_byte_timeout(deadline)
            if b is None:
                break
            buf.append(b)
            if buf.endswith(terminator):
                break
        return bytes(buf)

    def _read_byte_timeout(self, deadline: float) -> int | None:
        bt = self.bit_time
        pin = self._rx_pin
        req = self._rx_req

        # Wait for start bit with timeout
        while req.get_value(pin) == gpiod.line.Value.ACTIVE:
            if time.perf_counter() >= deadline:
                return None

        t = time.perf_counter() + bt * 1.5

        byte = 0
        for i in range(8):
            _busy_wait_until(t)
            if req.get_value(pin) == gpiod.line.Value.ACTIVE:
                byte |= (1 << i)
            t += bt

        _busy_wait_until(t)
        return byte

    def close(self):
        self._tx_req.release()
        self._rx_req.release()
        self._chip.close()


# Application logic (same flow as uart_test.py)

def read_from_esp(uart: BitBangUART):
    last_msg_time = time.time()
    avg_message_time = 0.0
    while True:
        data = uart.read_until(b'\n', timeout=0.5)
        if data:
            text = data.decode('ascii', errors='replace').strip()
            text = text.replace(';', '\n')
            print(f"[ESP32 -> Pi] {text}")

            current_time = time.time()
            elapsed = current_time - last_msg_time
            avg_message_time = (avg_message_time + elapsed) / 2 if avg_message_time > 0 else elapsed
            print(f"  Time since last message: {elapsed:.3f}s, Average: {avg_message_time:.3f}s")
            last_msg_time = current_time


def write_to_esp(uart: BitBangUART):
    num = 0
    while True:
        try:
            msg = f"{num},20,30,40,50"
            num = (num + 1) % 101
            time.sleep(0.1)
            packet = msg.encode("ascii")
            uart.write(packet)
            print(f"[Pi -> ESP32] Sent: {msg}")
        except KeyboardInterrupt:
            break


def main():
    print(f"Bit-bang UART on GPIO {TX_PIN} (TX) / GPIO {RX_PIN} (RX) @ {BAUD} baud")
    uart = BitBangUART()
    try:
        reader = threading.Thread(target=read_from_esp, args=(uart,), daemon=True)
        reader.start()
        write_to_esp(uart)
    finally:
        uart.close()


if __name__ == "__main__":
    main()
