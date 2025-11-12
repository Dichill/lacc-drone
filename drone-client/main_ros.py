#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Vector3
import cv_bridge

import picamera2
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
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
import os
import math
from typing import Optional, Dict, Tuple
from array import array

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from logger import get_logger, init_logger, close_logger

# --- ROS Bridge for Image Conversion ---
bridge = cv_bridge.CvBridge()

# --- Connection Configuration ---
# For SITL simulation, use TCP connection
# For real hardware, use serial: "/dev/serial0"
CONNECTION_STRING: str = "tcp:127.0.0.1:5763"  # SITL default
BAUD_RATE: int = 57600

# --- MQTT Configuration ---
BROKER_ADDRESS: str = "localhost"
BROKER_PORT: int = 1883
COMMAND_TOPIC: str = "drone/commands"
STREAM_TOPIC: str = "camera/stream"
ARUCO_TOPIC: str = "drone/aruco_detection"
TELEMETRY_TOPIC: str = "drone/telemetry"

# --- ArUco Configuration ---
# Using DICT_ARUCO_ORIGINAL to match the reference code
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
aruco_params = cv2.aruco.DetectorParameters_create()

# ArUco marker parameters
ID_TO_FIND: int = 72  # Target ArUco marker ID
MARKER_SIZE: float = 20.0  # Marker size in centimeters

# --- Camera Configuration ---
HORIZONTAL_RES: int = 640
VERTICAL_RES: int = 480
# Camera FOV for Raspberry Pi Camera V2
HORIZONTAL_FOV: float = 62.2 * (math.pi / 180.0)  # 62.2 degrees for Pi Cam V2
VERTICAL_FOV: float = 48.8 * (math.pi / 180.0)  # 48.8 degrees for Pi Cam V2

# --- Camera Calibration ---
camera_matrix: Optional[np.ndarray] = None
dist_coeffs: Optional[np.ndarray] = None
calibration_loaded: bool = False

# Default camera intrinsics (will be overridden if calibration file exists)
default_camera_matrix: np.ndarray = np.array([
    [530.8269276712998, 0.0, 320.5],
    [0.0, 530.8269276712998, 240.5],
    [0.0, 0.0, 1.0]
])
default_dist_coeffs: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Try to load camera calibration from file
calibration_file: str = "camera_calibration.npz"
try:
    if os.path.exists(calibration_file):
        calib_data = np.load(calibration_file)
        camera_matrix = calib_data["camera_matrix"]
        dist_coeffs = calib_data["dist_coeffs"]
        calibration_loaded = True
        print(f"✓ Camera calibration loaded from {calibration_file}")
    else:
        camera_matrix = default_camera_matrix
        dist_coeffs = default_dist_coeffs
        print(f"⚠ No camera calibration found. Using default intrinsics")
except Exception as e:
    camera_matrix = default_camera_matrix
    dist_coeffs = default_dist_coeffs
    print(f"⚠ Could not load calibration: {e}. Using default intrinsics")

# --- Flight Parameters ---
TAKEOFF_HEIGHT: float = 4.0  # meters
VELOCITY: float = 0.5  # m/s for horizontal movement
LAND_SPEED: int = 30  # cm/s

# --- Global State ---
centering_mode: bool = False  # Track if drone should center on ArUco marker
precision_land_mode: bool = False  # Track if in precision landing mode
vehicle = None  # Will be initialized in main
found_count: int = 0
notfound_count: int = 0
time_last: float = 0.0
time_to_wait: float = 0.1  # 100ms between detections

# --- ROS Publishers (Global) ---
aruco_detection_pub = None
telemetry_pub = None
image_pub = None


class StreamingOutput(io.BufferedIOBase):
    """Buffer for streaming camera frames."""
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf: bytes) -> int:
        """Write frame to buffer and notify waiting threads."""
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


def setup_vehicle_parameters(vehicle) -> None:
    """
    Configure vehicle parameters for precision landing.
    Sets up the parameters necessary for ArUco-based precision landing.
    """
    logger = get_logger()
    logger.info("Configuring vehicle parameters for precision landing")
    
    try:
        # Enable precision landing
        vehicle.parameters["PLND_ENABLED"] = 1
        logger.info("PLND_ENABLED set to 1")
        
        # Set precision landing type to companion computer
        vehicle.parameters["PLND_TYPE"] = 1
        logger.info("PLND_TYPE set to 1 (companion computer)")
        
        # Set precision landing estimator type
        vehicle.parameters["PLND_EST_TYPE"] = 0
        logger.info("PLND_EST_TYPE set to 0")
        
        # Set landing speed
        vehicle.parameters["LAND_SPEED"] = LAND_SPEED
        logger.info(f"LAND_SPEED set to {LAND_SPEED} cm/s")
        
        logger.event("VEHICLE_PARAMETERS_CONFIGURED")
        time.sleep(1)  # Allow parameters to settle
        
    except Exception as e:
        logger.error(f"Failed to set vehicle parameters: {e}", error=str(e))


def wait_for_armable(vehicle) -> None:
    """Wait until vehicle is armable."""
    logger = get_logger()
    logger.info("Waiting for vehicle to become armable")
    
    while not vehicle.is_armable:
        logger.info("Vehicle not armable yet, waiting...")
        time.sleep(1)
    
    logger.event("VEHICLE_ARMABLE")
    logger.info("Vehicle is now armable")


def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    """
    Arm vehicle and take off to target altitude.
    
    Args:
        vehicle: Dronekit vehicle object
        target_altitude: Target altitude in meters
    """
    logger = get_logger()
    
    # Wait until vehicle is armable
    wait_for_armable(vehicle)
    
    # Set mode to GUIDED
    logger.info("Setting flight mode to GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    
    while vehicle.mode.name != "GUIDED":
        logger.info("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    
    logger.event("FLIGHT_MODE_CHANGED", mode="GUIDED")
    logger.info("Vehicle now in GUIDED mode")
    
    # Arm the vehicle
    logger.info("Arming vehicle")
    logger.event("ARMING_MOTORS")
    vehicle.armed = True
    
    while not vehicle.armed:
        logger.info("Waiting for vehicle to become armed")
        time.sleep(1)
    
    logger.event("MOTORS_ARMED")
    logger.info("Vehicle armed! Props spinning!")
    
    # Take off
    logger.info(f"Taking off to {target_altitude} meters", target_altitude=target_altitude)
    logger.event("TAKEOFF_INITIATED", altitude=target_altitude)
    vehicle.simple_takeoff(target_altitude)
    
    # Wait until target altitude is reached
    while True:
        current_altitude: float = vehicle.location.global_relative_frame.alt
        logger.info(f"Current Altitude: {current_altitude:.2f}m", current_altitude=current_altitude)
        
        if current_altitude >= 0.95 * target_altitude:
            logger.event("TARGET_ALTITUDE_REACHED", target=target_altitude, current=current_altitude)
            logger.info("Target altitude reached!")
            break
        
        time.sleep(1)


def send_local_ned_velocity(vehicle, vx: float, vy: float, vz: float) -> None:
    """
    Send velocity command to vehicle in NED frame.
    
    Args:
        vehicle: Dronekit vehicle object
        vx: Velocity in x direction (North) in m/s
        vy: Velocity in y direction (East) in m/s
        vz: Velocity in z direction (Down) in m/s
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only velocity)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_land_message(vehicle, x_ang: float, y_ang: float) -> None:
    """
    Send landing target message to vehicle for precision landing.
    
    Args:
        vehicle: Dronekit vehicle object
        x_ang: Angular x position of target (radians)
        y_ang: Angular y position of target (radians)
    """
    msg = vehicle.message_factory.landing_target_encode(
        0,  # time_boot_ms (not used)
        0,  # target num
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        x_ang,  # angle x (radians)
        y_ang,  # angle y (radians)
        0,  # distance (meters) - 0 means unknown
        0, 0  # size x, size y (not used)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def move_to_marker_center(vehicle, center_x: float, center_y: float, frame_width: int, frame_height: int) -> None:
    """
    Move drone to center on ArUco marker using velocity commands.
    
    Args:
        vehicle: Dronekit vehicle object
        center_x: X coordinate of marker center in pixels
        center_y: Y coordinate of marker center in pixels
        frame_width: Width of camera frame in pixels
        frame_height: Height of camera frame in pixels
    """
    logger = get_logger()
    
    # Calculate offset from center (-1 to 1)
    offset_x: float = (center_x - frame_width / 2.0) / (frame_width / 2.0)
    offset_y: float = (center_y - frame_height / 2.0) / (frame_height / 2.0)
    
    # Apply deadzone to prevent small oscillations
    deadzone: float = 0.1
    if abs(offset_x) < deadzone:
        offset_x = 0.0
    if abs(offset_y) < deadzone:
        offset_y = 0.0
    
    # Convert to velocity commands
    velocity_gain: float = 0.5
    vx: float = offset_y * velocity_gain  # Forward/backward
    vy: float = offset_x * velocity_gain  # Left/right
    
    logger.info(f"Centering on marker - offset_x: {offset_x:.2f}, offset_y: {offset_y:.2f}, vx: {vx:.2f}, vy: {vy:.2f}",
               offset_x=offset_x, offset_y=offset_y, vx=vx, vy=vy)
    
    send_local_ned_velocity(vehicle, vx, vy, 0.0)


def process_aruco_detection(vehicle, img: np.ndarray, gray: np.ndarray, mqtt_client) -> Tuple[np.ndarray, Dict]:
    """
    Process frame for ArUco marker detection and handle precision landing.
    
    Args:
        vehicle: Dronekit vehicle object
        img: Original color image
        gray: Grayscale image for detection
        mqtt_client: MQTT client for publishing detection data
        
    Returns:
        Tuple of (processed image with overlays, detection data dictionary)
    """
    global found_count, notfound_count, time_last, time_to_wait, centering_mode, precision_land_mode
    logger = get_logger()
    
    # Initialize detection data
    detection_data: Dict = {
        "detected": False,
        "markers": [],
        "timestamp": time.time()
    }
    
    # Rate limit detection processing
    current_time: float = time.time()
    if current_time - time_last < time_to_wait:
        return img, detection_data
    
    time_last = current_time
    
    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    if ids is not None and len(ids) > 0:
        detection_data["detected"] = True
        
        # Check if target marker is detected
        target_found: bool = False
        target_idx: int = -1
        
        for i, marker_id in enumerate(ids):
            if marker_id[0] == ID_TO_FIND:
                target_found = True
                target_idx = i
                break
        
        if target_found:
            found_count += 1
            marker_corners = corners[target_idx][0]
            
            try:
                # Estimate pose of the marker
                ret = cv2.aruco.estimatePoseSingleMarkers(
                    corners[target_idx:target_idx+1],
                    MARKER_SIZE,
                    camera_matrix,
                    dist_coeffs
                )
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                
                # Get marker position in cm
                x_cm: float = tvec[0]
                y_cm: float = tvec[1]
                z_cm: float = tvec[2]
                
                # Calculate marker center in pixels
                x_sum: float = sum(corner[0] for corner in marker_corners)
                y_sum: float = sum(corner[1] for corner in marker_corners)
                x_avg: float = x_sum / 4.0
                y_avg: float = y_sum / 4.0
                
                # Calculate angular position (radians)
                x_ang: float = (x_avg - HORIZONTAL_RES * 0.5) * HORIZONTAL_FOV / HORIZONTAL_RES
                y_ang: float = (y_avg - VERTICAL_RES * 0.5) * VERTICAL_FOV / VERTICAL_RES
                
                # Calculate marker area
                marker_area: float = float(cv2.contourArea(marker_corners))
                
                # Store marker info
                marker_info: Dict = {
                    "id": int(ID_TO_FIND),
                    "center_x": float(x_avg),
                    "center_y": float(y_avg),
                    "area": marker_area,
                    "position_cm": {"x": float(x_cm), "y": float(y_cm), "z": float(z_cm)},
                    "angular_position": {"x": float(x_ang), "y": float(y_ang)},
                    "corners": marker_corners.tolist()
                }
                detection_data["markers"].append(marker_info)
                
                # Precision landing logic
                if precision_land_mode:
                    # Switch to LAND mode if not already
                    if vehicle.mode.name != "LAND":
                        logger.event("PRECISION_LAND_MODE_ACTIVATED")
                        logger.info("Switching to LAND mode for precision landing")
                        vehicle.mode = VehicleMode("LAND")
                        
                        while vehicle.mode.name != "LAND":
                            time.sleep(0.1)
                        
                        logger.info("Vehicle in LAND mode")
                    
                    # Send landing target message
                    send_land_message(vehicle, x_ang, y_ang)
                
                # Centering mode (hover and center on marker)
                elif centering_mode:
                    move_to_marker_center(vehicle, x_avg, y_avg, HORIZONTAL_RES, VERTICAL_RES)
                
                # Draw marker visualization
                cv2.aruco.drawDetectedMarkers(img, corners[target_idx:target_idx+1], ids[target_idx:target_idx+1])
                cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 10)
                
                # Draw marker information
                marker_position: str = f"MARKER: x={x_cm:.2f} y={y_cm:.2f} z={z_cm:.2f} cm"
                cv2.putText(img, marker_position, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), thickness=2)
                
                logger.info(marker_position, marker_id=ID_TO_FIND, x=x_cm, y=y_cm, z=z_cm)
                logger.info(f"FOUND: {found_count} | NOT FOUND: {notfound_count}", found=found_count, notfound=notfound_count)
                
            except Exception as e:
                logger.warning(f"Error processing marker: {e}", error=str(e))
                notfound_count += 1
        else:
            notfound_count += 1
            # Draw all detected markers even if not target
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
    else:
        notfound_count += 1
    
    # Publish detection data to MQTT
    detection_json: str = json.dumps(detection_data)
    mqtt_client.publish(ARUCO_TOPIC, detection_json, qos=0)
    
    # Publish to ROS topic if available
    if aruco_detection_pub is not None:
        aruco_msg = String()
        aruco_msg.data = detection_json
        aruco_detection_pub.publish(aruco_msg)
    
    return img, detection_data


def publish_telemetry(vehicle, mqtt_client) -> None:
    """
    Publish vehicle telemetry data to MQTT and ROS topics.
    
    Args:
        vehicle: Dronekit vehicle object
        mqtt_client: MQTT client for publishing
    """
    try:
        telemetry_data: Dict = {
            "mode": vehicle.mode.name,
            "armed": vehicle.armed,
            "altitude": vehicle.location.global_relative_frame.alt,
            "latitude": vehicle.location.global_relative_frame.lat,
            "longitude": vehicle.location.global_relative_frame.lon,
            "heading": vehicle.heading,
            "groundspeed": vehicle.groundspeed,
            "timestamp": time.time()
        }
        
        # Publish to MQTT
        telemetry_json: str = json.dumps(telemetry_data)
        mqtt_client.publish(TELEMETRY_TOPIC, telemetry_json, qos=0)
        
        # Publish to ROS if available
        if telemetry_pub is not None:
            telemetry_msg = String()
            telemetry_msg.data = telemetry_json
            telemetry_pub.publish(telemetry_msg)
            
    except Exception as e:
        logger = get_logger()
        logger.warning(f"Error publishing telemetry: {e}", error=str(e))


def process_command(command: str, vehicle) -> None:
    """
    Process incoming command from MQTT.
    
    Args:
        command: JSON command string
        vehicle: Dronekit vehicle object
    """
    global centering_mode, precision_land_mode
    logger = get_logger()
    logger.info("Processing command", command=command)
    
    try:
        data: Dict = json.loads(command)
        action: str = data.get("action", "")
        value: str = data.get("value", "")

        if action == "takeoff":
            logger.event("TAKEOFF_COMMAND")
            logger.info("Action: Executing 'takeoff' command")
            
            # Get target altitude from command, default to TAKEOFF_HEIGHT
            target_altitude: float = float(data.get("altitude", TAKEOFF_HEIGHT))
            logger.info(f"Target altitude set to {target_altitude} meters", target_altitude=target_altitude)
            
            # Execute arm and takeoff
            arm_and_takeoff(vehicle, target_altitude)
            logger.info("Takeoff complete")
            
        elif action == "land":
            logger.event("LAND_COMMAND")
            logger.info("Action: Executing 'land' command")
            centering_mode = False
            precision_land_mode = False
            vehicle.mode = VehicleMode("LAND")
            
            while vehicle.armed:
                logger.info("Waiting for vehicle to disarm...")
                time.sleep(1)
            
            logger.event("LANDING_COMPLETE")
            logger.info("Landing complete")
            
        elif action == "precision_land":
            logger.event("PRECISION_LAND_COMMAND")
            logger.info("Action: Enabling precision landing mode")
            centering_mode = False
            precision_land_mode = True
            
        elif action == "enable_centering":
            centering_mode = True
            precision_land_mode = False
            logger.event("CENTERING_MODE_ENABLED")
            logger.info("Centering mode enabled - drone will track ArUco marker")
            
        elif action == "disable_centering":
            centering_mode = False
            logger.event("CENTERING_MODE_DISABLED")
            logger.info("Centering mode disabled")
            
        elif action == "move_velocity":
            # Move drone with specified velocity
            vx: float = float(data.get("vx", 0.0))
            vy: float = float(data.get("vy", 0.0))
            vz: float = float(data.get("vz", 0.0))
            duration: float = float(data.get("duration", 1.0))
            
            logger.event("VELOCITY_COMMAND", vx=vx, vy=vy, vz=vz, duration=duration)
            logger.info(f"Moving with velocity - vx: {vx}, vy: {vy}, vz: {vz} for {duration}s")
            
            send_local_ned_velocity(vehicle, vx, vy, vz)
            time.sleep(duration)
            send_local_ned_velocity(vehicle, 0.0, 0.0, 0.0)  # Stop
            
        elif action == "move":
            logger.event("MOVE_COMMAND", value=value)
            logger.info(f"Moved to {value}", destination=value)
        else:
            logger.warning(f"Unknown command action: {action}", action=action)

    except json.JSONDecodeError as e:
        logger.warning("Received non-JSON message, ignoring", error=str(e))
    except Exception as e:
        logger.error(f"Error processing command: {e}", error=str(e), error_type=type(e).__name__)


def on_connect(client, userdata, flags, rc: int) -> None:
    """MQTT connection callback."""
    logger = get_logger()
    logger.event("MQTT_CONNECTED", result_code=rc)
    logger.info(f"Connected to MQTT Broker with result code {rc}", result_code=rc)
    client.subscribe(COMMAND_TOPIC)
    logger.info(f"Subscribed to topic: {COMMAND_TOPIC}", topic=COMMAND_TOPIC)


def on_message(client, userdata, msg) -> None:
    """MQTT message callback."""
    logger = get_logger()
    payload: str = msg.payload.decode()
    logger.info(f"[{msg.topic}] {payload}", topic=msg.topic, payload=payload)
    
    # Pass vehicle from userdata
    vehicle = userdata.get("vehicle") if isinstance(userdata, dict) else None
    if vehicle:
        process_command(payload, vehicle)


def ros_command_callback(msg: String) -> None:
    """
    ROS command callback - processes commands from ROS topic.
    
    Args:
        msg: ROS String message containing JSON command
    """
    logger = get_logger()
    logger.info("Received ROS command", command=msg.data)
    
    if vehicle:
        process_command(msg.data, vehicle)


def start_video_stream(vehicle, mqtt_client) -> None:
    """
    Start video stream with ArUco detection and precision landing.
    
    Args:
        vehicle: Dronekit vehicle object
        mqtt_client: MQTT client for publishing
    """
    logger = get_logger()
    logger.event("VIDEO_STREAM_START")

    now: datetime.datetime = datetime.datetime.now()
    timestamp_string: str = now.strftime("%Y-%m-%d_%H-%M-%S")
    video_file: str = f"./records/{timestamp_string}.mp4"

    logger.info("Starting video stream", video_file=video_file)

    with Picamera2() as camera:
        camera.configure(camera.create_video_configuration(main={"size": (HORIZONTAL_RES, VERTICAL_RES)}))
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
        skip_frames: int = 2  # Stream every 2nd frame
        telemetry_frames: int = 30  # Publish telemetry every 30 frames
        
        while True:
            with output3.condition:
                output3.condition.wait()
                frame = output3.frame

                if frame:
                    frame_count += 1
                    
                    # Decode frame
                    img_array: np.ndarray = np.frombuffer(frame, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    # Apply lens distortion correction if calibration is available
                    if calibration_loaded and camera_matrix is not None and dist_coeffs is not None:
                        img = cv2.undistort(img, camera_matrix, dist_coeffs)
                    
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    # Process ArUco detection
                    img, detection_data = process_aruco_detection(vehicle, img, gray, mqtt_client)
                    
                    # Publish frame to MQTT stream
                    if frame_count % skip_frames == 0:
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
                        ret, jpeg = cv2.imencode(".jpg", img, encode_param)
                        jpg_as_text: str = base64.b64encode(jpeg).decode("utf-8")
                        mqtt_client.publish(STREAM_TOPIC, jpg_as_text, qos=0)
                    
                    # Publish frame to ROS topic
                    if image_pub is not None:
                        try:
                            ros_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")
                            image_pub.publish(ros_image)
                        except Exception as e:
                            logger.warning(f"Error publishing ROS image: {e}")
                    
                    # Publish telemetry periodically
                    if frame_count % telemetry_frames == 0:
                        publish_telemetry(vehicle, mqtt_client)
                    
                    if frame_count % 100 == 0:
                        logger.metric("frames_streamed", frame_count, "frames")

                    # Check for quit command
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        logger.event("VIDEO_STREAM_STOP", frames_streamed=frame_count)
                        logger.info("Video stream stopped by user")
                        break

        output1.stop()
        logger.info("Video recording stopped")


def main() -> None:
    """Main entry point for ROS drone client."""
    global vehicle, aruco_detection_pub, telemetry_pub, image_pub
    
    logger = init_logger()
    logger.event("SYSTEM_START")
    logger.info("ROS Drone Client Starting")
    
    # Initialize ROS node
    try:
        rospy.init_node("drone_precision_landing_node", anonymous=False)
        logger.info("ROS node initialized: drone_precision_landing_node")
        
        # Create ROS publishers
        aruco_detection_pub = rospy.Publisher("/drone/aruco_detection", String, queue_size=10)
        telemetry_pub = rospy.Publisher("/drone/telemetry", String, queue_size=10)
        image_pub = rospy.Publisher("/camera/processed_image", Image, queue_size=10)
        
        # Create ROS subscriber for commands
        rospy.Subscriber("/drone/commands", String, ros_command_callback)
        
        logger.info("ROS publishers and subscribers created")
        
    except Exception as e:
        logger.warning(f"ROS initialization failed: {e}. Continuing with MQTT only.", error=str(e))
    
    # Connect to vehicle
    try:
        logger.info(f"Connecting to vehicle on {CONNECTION_STRING}")
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
        logger.event("VEHICLE_CONNECTED", connection=CONNECTION_STRING)
        logger.info("Vehicle connected successfully")
        
        # Setup precision landing parameters
        setup_vehicle_parameters(vehicle)
        
    except Exception as e:
        logger.error(f"Failed to connect to vehicle: {e}", error=str(e), error_type=type(e).__name__)
        close_logger()
        return
    
    # Setup MQTT client
    mqtt_client: mqtt.Client = mqtt.Client(userdata={"vehicle": vehicle})
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    try:
        logger.info(
            f"Connecting to MQTT broker at {BROKER_ADDRESS}:{BROKER_PORT}",
            broker=BROKER_ADDRESS,
            port=BROKER_PORT,
        )
        mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
        logger.event("MQTT_CONNECTION_INITIATED")
    except Exception as e:
        logger.error(
            f"Could not connect to MQTT broker: {e}",
            error=str(e),
            error_type=type(e).__name__,
        )
        vehicle.close()
        close_logger()
        return

    mqtt_client.loop_start()
    logger.info("MQTT loop started")

    try:
        start_video_stream(vehicle, mqtt_client)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(
            f"Video stream error: {e}", error=str(e), error_type=type(e).__name__
        )
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        vehicle.close()
        logger.event("SYSTEM_SHUTDOWN")
        logger.info("ROS Drone Client shutting down")
        close_logger()


if __name__ == "__main__":
    main()

