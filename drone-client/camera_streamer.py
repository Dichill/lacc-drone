import os
os.environ["LIBCAMERA_LOG_LEVELS"] = "ERROR"

import picamera2
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from threading import Condition
import io
import base64
import datetime
import numpy as np
import paho.mqtt.client as mqtt
import cv2
import json
import time
from typing import Optional, Dict, Any

broker_address = "localhost"
broker_port = 1883
stream_topic = "camera/stream"
aruco_topic = "drone/aruco_detection"
landing_target_topic = "drone/landing_target"
camera_status_topic = "camera/status"

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

camera_matrix: Optional[np.ndarray] = None
dist_coeffs: Optional[np.ndarray] = None
calibration_loaded: bool = False

calibration_file: str = "camera_calibration.npz"
try:
    if os.path.exists(calibration_file):
        calib_data = np.load(calibration_file)
        camera_matrix = calib_data["camera_matrix"]
        dist_coeffs = calib_data["dist_coeffs"]
        calibration_loaded = True
        print(f"✓ Camera calibration loaded from {calibration_file}")
    else:
        print(f"⚠ No camera calibration found")
except Exception as e:
    print(f"⚠ Could not load calibration: {e}")

mqtt_client: Optional[mqtt.Client] = None

last_stream_time: float = 0.0
stream_rate_limit: float = 0.033
last_aruco_time: float = 0.0
aruco_rate_limit: float = 0.1
last_aruco_publish_time: float = 0.0
aruco_publish_rate_limit: float = 0.1
last_detection_data: Dict[str, Any] = {"detected": False, "markers": [], "timestamp": time.time()}


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def on_connect(client, userdata, flags, rc: int) -> None:
    print(f"Connected to MQTT Broker with result code {rc}")
    print(f"Publishing to topics: {stream_topic}, {aruco_topic}")


def on_disconnect(client, userdata, rc: int) -> None:
    if rc != 0:
        print(f"Unexpected MQTT disconnect (code: {rc})")
    else:
        print("MQTT client disconnected")


def start_video_stream(client) -> None:
    global last_stream_time, last_aruco_time, last_aruco_publish_time, last_detection_data

    now: datetime.datetime = datetime.datetime.now()
    timestamp_string: str = now.strftime("%Y-%m-%d_%H-%M-%S")
    video_file: str = f"./records/{timestamp_string}.mp4"

    print(f"Starting camera streamer - recording to {video_file}")

    horizontal_res: int = 640
    vertical_res: int = 480
    horizontal_fov: float = 62.2 * (np.pi / 180)
    vertical_fov: float = 48.8 * (np.pi / 180)

    picamera2.Picamera2.set_logging(picamera2.Picamera2.ERROR)
    
    from picamera2.outputs import FfmpegOutput
    
    with Picamera2() as camera:
        try:
            print("Creating camera configuration...")
            config = camera.create_video_configuration(
                main={"size": (horizontal_res, vertical_res)},
                encode="main",
                controls={
                    "AfMode": 0,
                    "LensPosition": 0.0,
                    "FrameDurationLimits": (33333, 33333)
                }
            )
            
            print("Configuring camera...")
            camera.configure(config)
            print("Camera configured successfully")
            
            time.sleep(0.5)
            
            print("Setting camera controls...")
            camera.set_controls({"AfMode": 0, "LensPosition": 0.0})
            print("Camera controls set successfully")
            
        except Exception as e:
            print(f"Camera configuration failed: {e}")
            raise
        
        encoder: JpegEncoder = JpegEncoder()
        output1: FfmpegOutput = FfmpegOutput(video_file, audio=False)
        output3: StreamingOutput = StreamingOutput()
        output2: FileOutput = FileOutput(output3)
        encoder.output = [output1, output2]

        try:
            print("Starting camera encoder...")
            camera.start_encoder(encoder)
            print("Starting camera...")
            camera.start()
            print("Starting video output...")
            output1.start()
            
            time.sleep(1.0)
            print("✓ Camera streaming started")
            
            client.publish(camera_status_topic, json.dumps({"status": "running", "timestamp": time.time()}), qos=0)
            
        except Exception as e:
            print(f"Camera start failed: {e}")
            raise

        frame_count: int = 0
        first_frame_received: bool = False
        
        while True:
            with output3.condition:
                if not output3.condition.wait(timeout=5.0):
                    print("No frame received in 5 seconds")
                    continue
                frame = output3.frame

                if frame:
                    if not first_frame_received:
                        print("✓ First frame received!")
                        first_frame_received = True
                    
                    frame_count += 1
                    current_time: float = time.time()
                    
                    should_process_aruco: bool = (current_time - last_aruco_time) >= aruco_rate_limit
                    should_stream: bool = (current_time - last_stream_time) >= stream_rate_limit
                    
                    img_array: np.ndarray = np.frombuffer(frame, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if should_process_aruco:
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                        last_aruco_time = current_time
                    else:
                        ids = None
                        corners = None
                    
                    detection_data: Optional[Dict[str, Any]] = None
                    
                    if ids is not None and len(ids) > 0:
                        marker_id: int = int(ids[0][0])
                        marker_corners = corners[0][0]
                        
                        x_sum: float = sum(marker_corners[:, 0])
                        y_sum: float = sum(marker_corners[:, 1])
                        x_avg: float = x_sum / 4
                        y_avg: float = y_sum / 4
                        
                        x_ang: float = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                        y_ang: float = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res
                        
                        cv2.aruco.drawDetectedMarkers(img, corners, ids)
                        
                        detection_data = {
                            "detected": True,
                            "marker_id": marker_id,
                            "x_angle": float(x_ang),
                            "y_angle": float(y_ang),
                            "markers": [
                                {
                                    "id": marker_id,
                                    "center_x": float(x_avg),
                                    "center_y": float(y_avg),
                                    "corners": marker_corners.tolist()
                                }
                            ],
                            "timestamp": time.time()
                        }
                        
                        try:
                            client.publish(landing_target_topic, json.dumps({
                                "marker_id": marker_id,
                                "x_angle": float(x_ang),
                                "y_angle": float(y_ang),
                                "center_x": float(x_avg),
                                "center_y": float(y_avg),
                                "timestamp": time.time()
                            }), qos=0)
                        except Exception as e:
                            print(f"Landing target publish error: {e}")
                    else:
                        detection_data = {
                            "detected": False,
                            "markers": [],
                            "timestamp": time.time()
                        }
                    
                    if detection_data is not None:
                        last_detection_data = detection_data
                    
                    should_publish_aruco: bool = (current_time - last_aruco_publish_time) >= aruco_publish_rate_limit
                    
                    try:
                        if client and client.is_connected():
                            if should_publish_aruco or (detection_data is not None):
                                detection_json: str = json.dumps(last_detection_data)
                                client.publish(aruco_topic, detection_json, qos=0)
                                last_aruco_publish_time = current_time
                            
                            if should_stream:
                                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
                                ret, jpeg = cv2.imencode(".jpg", img, encode_param)
                                if ret:
                                    jpg_as_text: str = base64.b64encode(jpeg).decode("utf-8")
                                    client.publish(stream_topic, jpg_as_text, qos=0)
                                last_stream_time = current_time
                    except Exception as e:
                        print(f"MQTT publish error: {e}")

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print(f"Camera streamer stopped - {frame_count} frames processed")
                        break

        output1.stop()
        print("Video recording stopped")


def main() -> None:
    global mqtt_client
    
    print("=" * 60)
    print("LACC Drone - Camera Streamer")
    print("=" * 60)
    print(f"MQTT Broker: {broker_address}:{broker_port}")
    print("=" * 60)

    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)

    try:
        print(f"Connecting to MQTT broker at {broker_address}:{broker_port}...")
        mqtt_client.connect(broker_address, broker_port, 60)
        mqtt_client.loop_start()
        print("MQTT connected")
    except Exception as e:
        print(f"Could not connect to broker: {e}")
        return

    try:
        start_video_stream(mqtt_client)
    except Exception as e:
        print(f"Video stream error: {e}")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("Camera streamer shutting down")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        print("Disconnected")

