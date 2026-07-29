import time
import serial
from arduino_iot_cloud import ArduinoCloudClient

# Configuration
DEVICE_ID = "71fd9957-f4cf-44fa-a892-41b0da79909d"
SECRET_KEY = "9LQw1XF#beV9m#OeMN6IUM4L4"
COM_PORT = "COM4"
BAUD_RATE = 9600


def main():
    # 1. Setup Cloud
    client = ArduinoCloudClient(device_id=DEVICE_ID, username=DEVICE_ID, password=SECRET_KEY)

    variables = ["cloud_fanState", "cloud_humidity", "cloud_lightLevel",
                 "cloud_temperature", "cloud_ledState"]
    for var in variables:
        client.register(var)

    print("Connecting to Arduino Cloud...")
    client.start()

    # 2. Setup Serial
    print(f"Opening Serial port {COM_PORT}...")
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Error: Could not open port. {e}")
        return

    print("Bridge is running! Press Ctrl+C to stop.")

    # 3. Simple Loop
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith("DATA:"):
                    # Remove "DATA:" and split by comma
                    parts = line.replace("DATA:", "").split(",")

                    if len(parts) == 5:
                        try:
                            # Direct update to cloud variables
                            client['cloud_fanState'] = bool(float(parts[0]))
                            client['cloud_temperature'] = float(parts[1])
                            client['cloud_humidity'] = float(parts[2])
                            client['cloud_lightLevel'] = int(float(parts[3]))
                            client['cloud_ledState'] = bool(float(parts[4]))

                            print(f"Synced: Temp {parts[1]}C, Humidity {parts[2]}%")
                        except ValueError:
                            print("Skipping bad data line...")
                else:
                    # Print normal debug messages from Arduino
                    if line:
                        print(f"Arduino says: {line}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping bridge...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()