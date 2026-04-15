import serial
import threading
import time

PORT = "/dev/ttyAMA0"   # On Pi 5 with uart0-pi5 this is usually the GPIO UART. GPIO pins 14 (TX) and 15 (RX) are used for this UART. Make sure to connect TX to RX and RX to TX between the Pi and ESP32.
BAUD = 115200

def read_from_esp(ser):
    last_msg_time = time.time()
    avg_message_time = 0
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read_until(b'\n')
                if data:
                    data = data.decode('utf-8', errors='replace').strip()
                    data = data.replace(';', '\n')  # Replace semicolons with newlines for better readability
                    print(f"[ESP32 -> Pi] {data}")
                    # Timing analysis
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
            # msg = input("[Pi -> ESP32] Enter message (or '1', '2', ... for examples): ")
            msg = f"{num},20,30,40,50"  # Example message to send
            num = (num + 1) % 101  # Increment num for next message
            time.sleep(0.1)  # Simulate delay between messages
            if msg == "1":
                msg = "10,20,30,40,50"
                print(f" > Sending example message: {msg}")
            elif msg == "2":
                msg = "0,0,0,0,0"
                print(f" > Sending example message: {msg}")
            elif msg == "3":
                msg = "100,100,100,100,100"
                print(f" > Sending example message: {msg}")
            packet = (msg).encode("ascii")
            ser.write(packet)
            ser.flush()
        except serial.SerialException as e:
            print(f"Write error: {e}")
            break
        except KeyboardInterrupt:
            break

def main():
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            timeout=0.2,
            write_timeout=0.2
        )
        print(f"Opened {PORT} at {BAUD} baud")

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