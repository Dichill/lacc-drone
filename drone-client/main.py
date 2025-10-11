import picamera2  # camera module for RPi camera
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder, H264Encoder
from picamera2.outputs import FileOutput, FfmpegOutput
from threading import Condition
import io

import base64
import datetime
import numpy as np
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

# --- MQTT Configuration ---
broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
stream_topic = "camera/stream"


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


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
    now = datetime.datetime.now()
    timestamp_string = now.strftime("%Y-%m-%d_%H-%M-%S")

    with Picamera2() as camera:
        camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
        encoder = JpegEncoder()
        output1 = FfmpegOutput(
            f"{timestamp_string}.mp4", audio=False
        )  # Optional file recording
        output3 = StreamingOutput()
        output2 = FileOutput(output3)
        encoder.output = [output1, output2]

        # Start the camera and encoder
        camera.start_encoder(encoder)
        camera.start()
        output1.start()
        output1.stop()

        while True:
            with output3.condition:
                output3.condition.wait()
                frame = output3.frame

                if frame:
                    img_array = np.frombuffer(frame, dtype=np.uint8)

                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                    # Paola, this will be the grayscaled image which you would use for ArUco Marker Detection.
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    ret, jpeg = cv2.imencode(".jpg", img)
                    jpg_as_text = base64.b64encode(jpeg).decode("utf-8")

                    client.publish("camera/stream", jpg_as_text, qos=0)

                    time.sleep(0.1)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break


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
