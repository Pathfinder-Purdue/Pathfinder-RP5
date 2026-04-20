import serial
import serial.tools.list_ports
import threading
import time

BAUD = 9600


def find_usb_serial_port():
    """Auto-detect the USB-to-UART converter port."""
    ports = serial.tools.list_ports.comports()
    usb_ports = [p for p in ports if "USB" in p.device or "ACM" in p.device]
    if usb_ports:
        print("Detected USB serial ports:")
        for p in usb_ports:
            print(f"  {p.device} - {p.description}")
        return usb_ports[0].device
    # Fallback
    print("No USB serial port auto-detected, falling back to /dev/ttyUSB0")
    return "/dev/ttyUSB0"


def read_from_esp(ser):
    last_msg_time = time.time()
    avg_message_time = 0
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read_until(b'\n')
                if data:
                    data = data.decode('ascii', errors='replace').strip()
                    data = data.replace(';', '\n')
                    print(f"[ESP32 -> Pi] {data}")
                    current_time = time.time()
                    if last_msg_time is not None:
                        elapsed = current_time - last_msg_time
                        avg_message_time = (avg_message_time + elapsed) / 2 if avg_message_time > 0 else elapsed
                        print(f"  Time since last message: {elapsed:.3f} seconds, Average: {avg_message_time:.3f} seconds")
                    last_msg_time = current_time
            else:
                time.sleep(0.01)
        except serial.SerialException as e:
            print(f"Read error: {e}")
            break


def write_to_esp(ser):
    num = 0
    while True:
        try:
            msg = f"{num},{num},{num},{num},{num}"
            num = (num + 1) % 101
            time.sleep(0.1)
            packet = msg.encode("ascii")
            ser.write(packet)
            ser.flush()
            print(f"[Pi -> ESP32] Sent: {msg}")
        except serial.SerialException as e:
            print(f"Write error: {e}")
            break
        except KeyboardInterrupt:
            break


def main():
    port = find_usb_serial_port()
    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD,
            timeout=0.2,
            write_timeout=0.2
        )
        print(f"Opened {port} at {BAUD} baud")

        reader = threading.Thread(target=read_from_esp, args=(ser,), daemon=True)
        reader.start()

        write_to_esp(ser)

        reader.join()

    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
    finally:
        try:
            ser.close()
        except:
            pass


if __name__ == "__main__":
    main()
