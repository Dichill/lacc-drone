import picamera2
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

from pymavlink import mavutil
from logger import get_logger, init_logger, close_logger

# --- Connection to Pixhawk via serial port ---
# On a Raspberry Pi, the serial port is typically /dev/serial0
# The baud rate must match the SERIAL2_BAUD setting on your Pixhawk.
master = mavutil.mavlink_connection("/dev/serial0", baud=57600)

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


def process_command(command: str) -> None:
    logger = get_logger()
    logger.info("Processing command", command=command)
    try:
        data: dict = json.loads(command)
        action: str = data.get("action", "")
        value: str = data.get("value", "")

        if action == "takeoff":
            logger.event("TAKEOFF_COMMAND")
            logger.info("Action: Executing 'takeoff' command")
        elif action == "land":
            logger.event("LAND_COMMAND")
            logger.info("Action: Executing 'land' command")
            master.set_mode("LAND")

            time.sleep(1)

            while master.motors_armed():
                logger.info("Waiting for vehicle to disarm...")
                time.sleep(1)

            logger.event("LANDING_COMPLETE")
            logger.info("Landing complete")

        elif action == "move":
            logger.event("MOVE_COMMAND", value=value)
            logger.info(f"Moved to {value}", destination=value)
        else:
            logger.warning(f"Unknown command action: {action}", action=action)

    except json.JSONDecodeError as e:
        logger.warning("Received non-JSON message, ignoring", error=str(e))


def on_connect(client, userdata, flags, rc: int) -> None:
    logger = get_logger()
    logger.event("MQTT_CONNECTED", result_code=rc)
    logger.info(f"Connected to MQTT Broker with result code {rc}", result_code=rc)
    client.subscribe(command_topic)
    logger.info(f"Subscribed to topic: {command_topic}", topic=command_topic)


def on_message(client, userdata, msg) -> None:
    logger = get_logger()
    payload: str = msg.payload.decode()
    logger.info(f"[{msg.topic}] {payload}", topic=msg.topic, payload=payload)
    process_command(payload)


def start_video_stream(client) -> None:
    logger = get_logger()
    logger.event("VIDEO_STREAM_START")
    
    now: datetime.datetime = datetime.datetime.now()
    timestamp_string: str = now.strftime("%Y-%m-%d_%H-%M-%S")
    video_file: str = f"./records/{timestamp_string}.mp4"
    
    logger.info("Starting video stream", video_file=video_file)

    with Picamera2() as camera:
        camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
        encoder: JpegEncoder = JpegEncoder()
        output1: FfmpegOutput = FfmpegOutput(video_file, audio=False)
        output3: StreamingOutput = StreamingOutput()
        output2: FileOutput = FileOutput(output3)
        encoder.output = [output1, output2]

        camera.start_encoder(encoder)
        camera.start()
        output1.start()
        logger.info("Camera encoder started")
        
        time.sleep(20)
        output1.stop()
        logger.info("Video recording stopped")

        frame_count: int = 0
        while True:
            with output3.condition:
                output3.condition.wait()
                frame = output3.frame

                if frame:
                    img_array: np.ndarray = np.frombuffer(frame, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    ret, jpeg = cv2.imencode(".jpg", img)
                    jpg_as_text: str = base64.b64encode(jpeg).decode("utf-8")

                    client.publish("camera/stream", jpg_as_text, qos=0)
                    
                    frame_count += 1
                    if frame_count % 100 == 0:
                        logger.metric("frames_streamed", frame_count, "frames")

                    time.sleep(0.1)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        logger.event("VIDEO_STREAM_STOP", frames_streamed=frame_count)
                        logger.info("Video stream stopped by user")
                        break


def main() -> None:
    logger = init_logger()
    logger.event("SYSTEM_START")
    logger.info("Drone client starting")
    
    client: mqtt.Client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        logger.info(f"Connecting to MQTT broker at {broker_address}:{broker_port}", 
                   broker=broker_address, port=broker_port)
        client.connect(broker_address, broker_port, 60)
        logger.event("MQTT_CONNECTION_INITIATED")
    except Exception as e:
        logger.error(f"Could not connect to broker: {e}", error=str(e), error_type=type(e).__name__)
        close_logger()
        return

    client.loop_start()
    logger.info("MQTT loop started")

    try:
        start_video_stream(client)
    except Exception as e:
        logger.error(f"Video stream error: {e}", error=str(e), error_type=type(e).__name__)
    finally:
        client.loop_stop()
        client.disconnect()
        logger.event("SYSTEM_SHUTDOWN")
        logger.info("Drone client shutting down")
        close_logger()


if __name__ == "__main__":
    main()
