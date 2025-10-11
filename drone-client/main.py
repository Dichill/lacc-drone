import base64
import paho.mqtt.client as mqtt
import cv2
import json
import time

"""

    MAVLINK

"""
# from pymavlink import mavutil

# --- Connection to Pixhawk via serial port ---
# On a Raspberry Pi, the serial port is typically /dev/serial0
# The baud rate must match the SERIAL2_BAUD setting on your Pixhawk.
# master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
import base64
import paho.mqtt.client as mqtt
import cv2
import json
import time

# --- MQTT Configuration ---
broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
stream_topic = "camera/stream"


def process_command(command):
    """Parses a command and takes action on the drone."""
    print(f"Processing command: {command}")
    try:
        data = json.loads(command)
        action = data.get("action")
        value = data.get("value")

        if action == "takeoff":
            print("Action: Executing 'takeoff' command.")
            # mavutil.set_mode_auto()  # Example MAVLink command
        elif action == "land":
            print("Action: Executing 'land' command.")
            # Add MAVLink land command here
        elif action == "move":
            print(f"Moved to {value}")
            # Add MAVLink movement command here
        else:
            print(f"Unknown command action: {action}")

    except json.JSONDecodeError:
        print("Received non-JSON message, ignoring.")


def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    print(f"Connected to MQTT Broker with result code {rc}")
    client.subscribe(command_topic)
    print(f"Subscribed to topic: {command_topic}")


def on_message(client, userdata, msg):
    """Callback for when a message is received on a subscribed topic."""
    payload = msg.payload.decode()
    print(f"[{msg.topic}] {payload}")
    process_command(payload)


def start_video_stream(client):
    """Handles video capture and streaming."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video device.")
        return

    print("Starting video stream...")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            _, buffer = cv2.imencode(".jpg", frame)
            jpg_as_text = base64.b64encode(buffer).decode("utf-8")

            client.publish(stream_topic, jpg_as_text, qos=0)

            time.sleep(0.1)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("Stopping video stream.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(broker_address, broker_port, 60)
    except Exception as e:
        print(f"Could not connect to broker: {e}")
        return

    client.loop_start()

    start_video_stream(client)

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
