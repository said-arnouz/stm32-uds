import serial
import time
PORT = "COM4"  
BAUD = 115200

def parse_input(user_input: str):
    # Clean input
    data = user_input.upper().replace(" ", "")

    # Remove 0x
    if data.startswith("0X"):
        data = data[2:]

    # Convert to bytes
    bytes_list = []
    for i in range(0, len(data), 2):
        byte = int(data[i:i+2], 16)
        bytes_list.append(byte)

    return bytes_list


def build_frame(payload):
    length = len(payload)
    frame = [length] + payload  # pas de padding
        # padding to 8 bytes
    while len(frame) < 8:
        frame.append(0xAA)
    return bytes(frame)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)

    print("=== UDS UART HOST ===")

    while True:
        try:
            user_input = input("Tester: ")

            payload = parse_input(user_input)
            frame = build_frame(payload)
            ser.write(frame)
            time.sleep(0.05)
            # Read response (8 bytes)
            resp = ser.read(8)

            if resp:
                print("ECU:", resp.hex().upper())
            else:
                print("No response")

        except Exception as e:
            print("Error:", e)


if __name__ == "__main__":
    main()