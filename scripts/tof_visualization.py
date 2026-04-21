"""
Live 3D bar chart of the 4x4 ToF sensor grid.

Reads ESP32 UART packets over USB serial (same method as uart_usb.py),
parses the 16 TOF values, and displays them as a 4x4 3D bar graph
with index labels.
"""

import serial
import serial.tools.list_ports
import threading
import time

import matplotlib.pyplot as plt
import numpy as np

BAUD = 115200


def find_usb_serial_port():
    # optional auto-detection code for USB UART port
    # ports = serial.tools.list_ports.comports()
    # usb_ports = [p for p in ports if "USB" in p.device or "ACM" in p.device]
    # if usb_ports:
    #     print("Detected USB serial ports:")
    #     for p in usb_ports:
    #         print(f"  {p.device} - {p.description}")
    #     return usb_ports[0].device
    # print("No USB serial port auto-detected, falling back to /dev/ttyUSB0")
    # return "/dev/ttyUSB0"
    return "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0"


def parse_tof(msg):
    """Extract 16 ToF values from a semicolon-delimited UART message."""
    for part in msg.split(';'):
        part = part.strip()
        if part.startswith('TOF:'):
            try:
                vals = [int(v.strip()) for v in part[4:].split(',')]
                if len(vals) == 16:
                    return vals
            except (ValueError, IndexError):
                pass
    return None


def reader_thread(ser, tof_state, lock):
    """Background thread that reads UART and updates tof_state."""
    while True:
        try:
            if ser.in_waiting > 0:
                raw = ser.read_until(b'\n')
                if raw:
                    msg = raw.decode('ascii', errors='replace').strip()
                    tof = parse_tof(msg)
                    if tof is not None:
                        with lock:
                            tof_state[:] = tof
            else:
                time.sleep(0.005)
        except serial.SerialException as e:
            print(f"Read error: {e}")
            break


def main():
    port = find_usb_serial_port()
    ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.2, write_timeout=0.2)
    print(f"Opened {port} at {BAUD} baud")

    tof_state = [0] * 16
    lock = threading.Lock()

    t = threading.Thread(target=reader_thread, args=(ser, tof_state, lock), daemon=True)
    t.start()

    # set up 3D bar chart
    plt.ion()
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # bar positions for a 4x4 grid
    xpos, ypos = np.meshgrid(np.arange(4), np.arange(4))
    xpos = xpos.flatten()
    ypos = ypos.flatten()
    zpos = np.zeros(16)
    dx = dy = 0.6

    try:
        while True:
            with lock:
                vals = list(tof_state)

            ax.cla()
            dz = np.array(vals, dtype=float)

            colors = plt.cm.RdYlGn(1.0 - np.clip(dz / 4000.0, 0, 1))
            ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color=colors, alpha=0.85)

            # label each bar with its index and value
            for i in range(16):
                ax.text(xpos[i] + dx / 2, ypos[i] + dy / 2, dz[i] + 40,
                        f"[{i}]\n{vals[i]}", ha='center', va='bottom', fontsize=7)

            ax.set_xlabel('Column')
            ax.set_ylabel('Row')
            ax.set_zlabel('Distance (mm)')
            ax.set_xticks(np.arange(4) + dx / 2)
            ax.set_xticklabels(['0', '1', '2', '3'])
            ax.set_yticks(np.arange(4) + dy / 2)
            ax.set_yticklabels(['0', '1', '2', '3'])
            ax.set_zlim(0, 4500)
            ax.set_title('ToF 4x4 Sensor Grid (live)')

            plt.draw()
            plt.pause(0.15)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
