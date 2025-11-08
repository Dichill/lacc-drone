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
aruco_topic = "drone/aruco_detection"

# --- ArUco Configuration ---
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

# --- Global State ---
centering_mode: bool = False  # Track if drone should center on ArUco marker


def wait_for_ready(master) -> None:
    logger = get_logger()
    logger.info("Waiting for heartbeat...")
    master.wait_heartbeat()
    logger.event("HEARTBEAT_RECEIVED", 
                system=master.target_system, 
                component=master.target_component)
    logger.info(f"Heartbeat from system (system {master.target_system} component {master.target_component})",
               system=master.target_system, 
               component=master.target_component)
    time.sleep(2)


def arm_and_takeoff(master, target_altitude: float) -> None:
    logger = get_logger()
    
    # Check if already armed
    if master.motors_armed():
        logger.info("Vehicle is already armed")
    else:
        logger.info("Waiting for arming...")
        logger.event("ARMING_MOTORS")
        master.arducopter_arm()
        time.sleep(2)
        logger.event("MOTORS_ARMED")

    # Set flight mode to GUIDED
    logger.info("Setting flight mode to GUIDED...")
    master.set_mode("GUIDED")
    logger.event("FLIGHT_MODE_CHANGED", mode="GUIDED")
    time.sleep(1)

    # Send takeoff command
    logger.info(f"Taking off to {target_altitude} meters...", target_altitude=target_altitude)
    logger.event("TAKEOFF_INITIATED", altitude=target_altitude)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        target_altitude,
    )


def wait_for_altitude(master, target_altitude: float, tolerance: float = 0.5) -> None:
    """Wait until the drone reaches the target altitude."""
    logger = get_logger()
    logger.info("Waiting for altitude...", target_altitude=target_altitude, tolerance=tolerance)
    start_time: float = time.time()
    
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if not msg:
            continue

        current_altitude: float = msg.relative_alt / 1000.0
        logger.metric("current_altitude", current_altitude, "meters")
        logger.info(f"Altitude: {current_altitude:.2f}m", current_altitude=current_altitude)

        if abs(current_altitude - target_altitude) < tolerance:
            logger.event("TARGET_ALTITUDE_REACHED", 
                        target=target_altitude, 
                        current=current_altitude)
            logger.info("Target altitude reached")
            break

        elapsed_time: float = time.time() - start_time
        if elapsed_time > 30:
            logger.warning("Timed out waiting for altitude", 
                          elapsed_time=elapsed_time,
                          current_altitude=current_altitude)
            break

        time.sleep(1)


def move_to_marker_center(master, center_x: float, center_y: float, frame_width: int, frame_height: int) -> None:
    logger = get_logger()
    
    offset_x: float = (center_x - frame_width / 2) / (frame_width / 2)
    offset_y: float = (center_y - frame_height / 2) / (frame_height / 2)
    
    deadzone: float = 0.1
    if abs(offset_x) < deadzone:
        offset_x = 0.0
    if abs(offset_y) < deadzone:
        offset_y = 0.0
    
    velocity_gain: float = 0.5
    vx: float = offset_y * velocity_gain  
    vy: float = offset_x * velocity_gain 
    
    logger.info(f"Adjusting position - offset_x: {offset_x:.2f}, offset_y: {offset_y:.2f}, vx: {vx:.2f}, vy: {vy:.2f}",
               offset_x=offset_x, offset_y=offset_y, vx=vx, vy=vy)
    
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # type_mask (only velocity)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, 0,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )


def auto_land_on_marker(master) -> None:
    """Switch to LAND mode for auto-landing on ArUco marker."""
    logger = get_logger()
    logger.event("AUTO_LAND_INITIATED")
    logger.info("Initiating auto-landing on ArUco marker...")
    
    master.set_mode("LAND")
    logger.event("FLIGHT_MODE_CHANGED", mode="LAND")
    time.sleep(1)
    
    while master.motors_armed():
        logger.info("Landing in progress...")
        time.sleep(1)
    
    logger.event("AUTO_LAND_COMPLETE")
    logger.info("Auto-landing complete")


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def process_command(command: str) -> None:
    global centering_mode
    logger = get_logger()
    logger.info("Processing command", command=command)
    try:
        data: dict = json.loads(command)
        action: str = data.get("action", "")
        value: str = data.get("value", "")

        if action == "takeoff":
            logger.event("TAKEOFF_COMMAND")
            logger.info("Action: Executing 'takeoff' command")
            
            # Get target altitude from command, default to 5.0 meters
            target_altitude: float = float(data.get("altitude", 5.0))
            logger.info(f"Target altitude set to {target_altitude} meters", target_altitude=target_altitude)
            
            # Wait for flight controller to be ready
            wait_for_ready(master)
            
            # Execute arm and takeoff
            arm_and_takeoff(master, target_altitude)
            
            # Wait for target altitude to be reached
            wait_for_altitude(master, target_altitude)
            
            logger.info("Takeoff complete")
            
        elif action == "land":
            logger.event("LAND_COMMAND")
            logger.info("Action: Executing 'land' command")
            centering_mode = False  # Disable centering mode
            master.set_mode("LAND")

            time.sleep(1)

            while master.motors_armed():
                logger.info("Waiting for vehicle to disarm...")
                time.sleep(1)

            logger.event("LANDING_COMPLETE")
            logger.info("Landing complete")

        elif action == "auto_land":
            logger.event("AUTO_LAND_COMMAND")
            logger.info("Action: Executing 'auto_land' command - will land when centered on marker")
            centering_mode = False  # Disable centering, just land
            auto_land_on_marker(master)
            
        elif action == "enable_centering":
            centering_mode = True
            logger.event("CENTERING_MODE_ENABLED")
            logger.info("Centering mode enabled - drone will track ArUco marker")
            
        elif action == "disable_centering":
            centering_mode = False
            logger.event("CENTERING_MODE_DISABLED")
            logger.info("Centering mode disabled")

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
    global centering_mode
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

        frame_count: int = 0
        frame_width: int = 640
        frame_height: int = 480
        
        while True:
            with output3.condition:
                output3.condition.wait()
                frame = output3.frame

                if frame:
                    img_array: np.ndarray = np.frombuffer(frame, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                    
                    detection_data: dict = {
                        "detected": False,
                        "markers": [],
                        "timestamp": time.time()
                    }
                    
                    if ids is not None and len(ids) > 0:
                        detection_data["detected"] = True
                        
                        for i, marker_id in enumerate(ids):
                            marker_corners = corners[i][0]
                            
                            center_x: float = float(np.mean(marker_corners[:, 0]))
                            center_y: float = float(np.mean(marker_corners[:, 1]))
                            
                            marker_area: float = float(cv2.contourArea(marker_corners))
                            
                            marker_info: dict = {
                                "id": int(marker_id[0]),
                                "center_x": center_x,
                                "center_y": center_y,
                                "area": marker_area,
                                "corners": marker_corners.tolist()
                            }
                            detection_data["markers"].append(marker_info)
                            
                            logger.info(f"ArUco marker detected - ID: {marker_id[0]}, Center: ({center_x:.1f}, {center_y:.1f})",
                                       marker_id=int(marker_id[0]), center_x=center_x, center_y=center_y)
                            
                            if centering_mode and i == 0:  # Only center on first marker
                                move_to_marker_center(master, center_x, center_y, frame_width, frame_height)
                        
                        cv2.aruco.drawDetectedMarkers(img, corners, ids)
                    
                    detection_json: str = json.dumps(detection_data)
                    client.publish(aruco_topic, detection_json, qos=0)
                    
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                    ret, jpeg = cv2.imencode(".jpg", img, encode_param)
                    jpg_as_text: str = base64.b64encode(jpeg).decode("utf-8")
                    client.publish(stream_topic, jpg_as_text, qos=0)

                    frame_count += 1
                    if frame_count % 100 == 0:
                        logger.metric("frames_streamed", frame_count, "frames")

                    time.sleep(0.033)

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        logger.event("VIDEO_STREAM_STOP", frames_streamed=frame_count)
                        logger.info("Video stream stopped by user")
                        break

        output1.stop()
        logger.info("Video recording stopped")


def main() -> None:
    logger = init_logger()
    logger.event("SYSTEM_START")
    logger.info("Drone client starting")

    client: mqtt.Client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        logger.info(
            f"Connecting to MQTT broker at {broker_address}:{broker_port}",
            broker=broker_address,
            port=broker_port,
        )
        client.connect(broker_address, broker_port, 60)
        logger.event("MQTT_CONNECTION_INITIATED")
    except Exception as e:
        logger.error(
            f"Could not connect to broker: {e}",
            error=str(e),
            error_type=type(e).__name__,
        )
        close_logger()
        return

    client.loop_start()
    logger.info("MQTT loop started")

    try:
        start_video_stream(client)
    except Exception as e:
        logger.error(
            f"Video stream error: {e}", error=str(e), error_type=type(e).__name__
        )
    finally:
        client.loop_stop()
        client.disconnect()
        logger.event("SYSTEM_SHUTDOWN")
        logger.info("Drone client shutting down")
        close_logger()


if __name__ == "__main__":
    main()
