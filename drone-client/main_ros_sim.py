#!/usr/bin/env python
"""
ROS-Compatible Drone Client with OpenCV Camera Simulation
This version uses OpenCV VideoCapture for simulation and integrates with ROS
Compatible with both Python 2.7 (ROS Melodic) and Python 3.x (ROS Noetic)
"""

from __future__ import print_function
from __future__ import division

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import base64
import datetime
import numpy as np
import paho.mqtt.client as mqtt
import cv2
import json
import time
import os
import sys

try:
    import cv2.aruco as aruco
except ImportError:
    print("Warning: Could not import cv2.aruco, using cv2.aruco module")
    aruco = cv2.aruco

from pymavlink import mavutil

# Import logger with fallback for Python 2.7
try:
    from logger import get_logger, init_logger, close_logger
except ImportError:
    print("Warning: Logger module not available, using basic logging")
    class DummyLogger:
        def info(self, *args, **kwargs):
            print("[INFO]", args)
        def warning(self, *args, **kwargs):
            print("[WARNING]", args)
        def error(self, *args, **kwargs):
            print("[ERROR]", args)
        def event(self, *args, **kwargs):
            print("[EVENT]", args)
        def metric(self, *args, **kwargs):
            print("[METRIC]", args)
    
    def get_logger():
        return DummyLogger()
    
    def init_logger():
        return DummyLogger()
    
    def close_logger():
        pass

# --- Connection to Pixhawk (can use TCP for SITL simulation) ---
# For SITL: "tcp:127.0.0.1:5763"
# For hardware: "/dev/serial0" with baud=57600
connection_string = "tcp:127.0.0.1:5763"  # Default for SITL simulation
use_sitl = True

try:
    master = mavutil.mavlink_connection(connection_string)
    print("Connecting to vehicle on: {}".format(connection_string))
except Exception as e:
    print("Error connecting to vehicle: {}".format(e))
    master = None

# --- MQTT Configuration ---
broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
stream_topic = "camera/stream"
aruco_topic = "drone/aruco_detection"

# --- ArUco Configuration ---
try:
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = aruco.DetectorParameters_create()
except AttributeError:
    # For newer OpenCV versions
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = aruco.DetectorParameters()

# --- Camera Configuration ---
camera_index = 0  # Default camera (0 for built-in webcam, or path to video file)
frame_width = 640
frame_height = 480

# --- Camera Calibration ---
camera_matrix = None
dist_coeffs = None
calibration_loaded = False

# Camera intrinsics from your ROS code
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix_default = [[530.8269276712998, 0.0, 320.5],
                        [0.0, 530.8269276712998, 240.5],
                        [0.0, 0.0, 1.0]]
camera_matrix = np.array(camera_matrix_default)
dist_coeffs = np.array(dist_coeff)

# Try to load camera calibration from file
calibration_file = "camera_calibration.npz"
try:
    if os.path.exists(calibration_file):
        calib_data = np.load(calibration_file)
        camera_matrix = calib_data["camera_matrix"]
        dist_coeffs = calib_data["dist_coeffs"]
        calibration_loaded = True
        print("Camera calibration loaded from {}".format(calibration_file))
    else:
        print("No camera calibration found. Using default values")
        calibration_loaded = True
except Exception as e:
    print("Could not load calibration: {}".format(e))
    calibration_loaded = True

# --- Global State ---
centering_mode = False  # Track if drone should center on ArUco marker
landing_mode = False  # Track if drone is in landing mode
id_to_find = 72  # ArUco marker ID to track
marker_size = 20  # Marker size in CM

# --- ROS Publishers ---
image_pub = None
aruco_image_pub = None

# --- Camera FOV (from your ROS code) ---
import math
horizontal_fov = 62.2 * (math.pi / 180)  # 62.2 for picam V2
vertical_fov = 48.8 * (math.pi / 180)  # 48.8 for V2


def wait_for_ready(master):
    """Wait for heartbeat from flight controller."""
    logger = get_logger()
    logger.info("Waiting for heartbeat...")
    if master:
        master.wait_heartbeat()
        logger.event("HEARTBEAT_RECEIVED")
        logger.info("Heartbeat from system {} component {}".format(
            master.target_system, master.target_component))
    time.sleep(2)


def arm_and_takeoff(master, target_altitude):
    """Arm motors and takeoff to target altitude."""
    logger = get_logger()
    
    if not master:
        logger.error("No connection to vehicle")
        return
    
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
    logger.event("FLIGHT_MODE_CHANGED")
    time.sleep(1)

    # Send takeoff command
    logger.info("Taking off to {} meters...".format(target_altitude))
    logger.event("TAKEOFF_INITIATED")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0,
        target_altitude,
    )


def wait_for_altitude(master, target_altitude, tolerance=0.5):
    """Wait until the drone reaches the target altitude."""
    logger = get_logger()
    logger.info("Waiting for altitude...")
    start_time = time.time()
    
    if not master:
        return
    
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if not msg:
            continue

        current_altitude = msg.relative_alt / 1000.0
        logger.info("Altitude: {:.2f}m".format(current_altitude))

        if abs(current_altitude - target_altitude) < tolerance:
            logger.event("TARGET_ALTITUDE_REACHED")
            logger.info("Target altitude reached")
            break

        elapsed_time = time.time() - start_time
        if elapsed_time > 30:
            logger.warning("Timed out waiting for altitude")
            break

        time.sleep(1)


def send_land_message(master, x_ang, y_ang):
    """
    Send landing target message to flight controller.
    x_ang, y_ang: angular offsets in radians
    """
    if not master:
        return
    
    msg = master.mav.landing_target_encode(
        0,  # time_boot_ms
        0,  # target_num
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x_ang,
        y_ang,
        0, 0, 0  # distance, size_x, size_y (not used)
    )
    master.send_mavlink(msg)
    master.flush()


def move_to_marker_center(master, center_x, center_y, frame_width, frame_height):
    """Move drone to center on ArUco marker using velocity commands."""
    logger = get_logger()
    
    if not master:
        return
    
    # Calculate offset from center (-1 to 1)
    offset_x = (center_x - frame_width / 2) / (frame_width / 2)
    offset_y = (center_y - frame_height / 2) / (frame_height / 2)
    
    # Apply deadzone
    deadzone = 0.1
    if abs(offset_x) < deadzone:
        offset_x = 0.0
    if abs(offset_y) < deadzone:
        offset_y = 0.0
    
    # Calculate velocity commands
    velocity_gain = 0.5
    vx = offset_y * velocity_gain  
    vy = offset_x * velocity_gain 
    
    logger.info("Adjusting position - offset_x: {:.2f}, offset_y: {:.2f}, vx: {:.2f}, vy: {:.2f}".format(
        offset_x, offset_y, vx, vy))
    
    # Send velocity command
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


def auto_land_on_marker(master):
    """Switch to LAND mode for auto-landing on ArUco marker."""
    global landing_mode
    logger = get_logger()
    logger.event("AUTO_LAND_INITIATED")
    logger.info("Initiating auto-landing on ArUco marker...")
    
    if not master:
        return
    
    landing_mode = True
    master.set_mode("LAND")
    logger.event("FLIGHT_MODE_CHANGED")
    time.sleep(1)


def process_command(command):
    """Process incoming MQTT command."""
    global centering_mode, landing_mode
    logger = get_logger()
    logger.info("Processing command: {}".format(command))
    
    try:
        data = json.loads(command)
        action = data.get("action", "")
        value = data.get("value", "")

        if action == "takeoff":
            logger.event("TAKEOFF_COMMAND")
            logger.info("Action: Executing 'takeoff' command")
            
            # Get target altitude from command, default to 5.0 meters
            target_altitude = float(data.get("altitude", 5.0))
            logger.info("Target altitude set to {} meters".format(target_altitude))
            
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
            centering_mode = False
            landing_mode = False
            if master:
                master.set_mode("LAND")
            time.sleep(1)

        elif action == "auto_land":
            logger.event("AUTO_LAND_COMMAND")
            logger.info("Action: Executing 'auto_land' command")
            centering_mode = False
            auto_land_on_marker(master)
            
        elif action == "enable_centering":
            centering_mode = True
            logger.event("CENTERING_MODE_ENABLED")
            logger.info("Centering mode enabled")
            
        elif action == "disable_centering":
            centering_mode = False
            logger.event("CENTERING_MODE_DISABLED")
            logger.info("Centering mode disabled")

        elif action == "move":
            logger.event("MOVE_COMMAND")
            logger.info("Moved to {}".format(value))
        else:
            logger.warning("Unknown command action: {}".format(action))

    except (json.JSONDecodeError, ValueError) as e:
        logger.warning("Received invalid message: {}".format(e))


def on_connect(client, userdata, flags, rc):
    """MQTT connection callback."""
    logger = get_logger()
    logger.event("MQTT_CONNECTED")
    logger.info("Connected to MQTT Broker with result code {}".format(rc))
    client.subscribe(command_topic)
    logger.info("Subscribed to topic: {}".format(command_topic))


def on_message(client, userdata, msg):
    """MQTT message callback."""
    logger = get_logger()
    payload = msg.payload.decode()
    logger.info("[{}] {}".format(msg.topic, payload))
    process_command(payload)


def process_aruco_detection(img, gray, corners, ids, frame_width, frame_height):
    """
    Process detected ArUco markers and send commands to drone.
    Returns: annotated image with markers drawn
    """
    global landing_mode
    logger = get_logger()
    
    detection_data = {
        "detected": False,
        "markers": [],
        "timestamp": time.time()
    }
    
    if ids is not None and len(ids) > 0:
        detection_data["detected"] = True
        
        for i, marker_id in enumerate(ids):
            marker_corners = corners[i][0]
            
            # Calculate center of marker
            center_x = float(np.mean(marker_corners[:, 0]))
            center_y = float(np.mean(marker_corners[:, 1]))
            
            # Calculate marker area
            marker_area = float(cv2.contourArea(marker_corners))
            
            # Store marker info
            marker_info = {
                "id": int(marker_id[0]),
                "center_x": center_x,
                "center_y": center_y,
                "area": marker_area,
                "corners": marker_corners.tolist()
            }
            detection_data["markers"].append(marker_info)
            
            # If this is the target marker
            if marker_id[0] == id_to_find:
                logger.info("ArUco marker detected - ID: {}, Center: ({:.1f}, {:.1f})".format(
                    marker_id[0], center_x, center_y))
                
                # Calculate angular offsets for precision landing
                x_sum = sum([corner[0] for corner in marker_corners])
                y_sum = sum([corner[1] for corner in marker_corners])
                x_avg = x_sum / 4
                y_avg = y_sum / 4
                
                x_ang = (x_avg - frame_width * 0.5) * horizontal_fov / frame_width
                y_ang = (y_avg - frame_height * 0.5) * vertical_fov / frame_height
                
                # If in landing mode, send landing target
                if landing_mode and master:
                    send_land_message(master, x_ang, y_ang)
                    
                    # Draw landing info on image
                    cv2.putText(img, "LANDING MODE", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # If in centering mode, adjust position
                elif centering_mode and i == 0:
                    move_to_marker_center(master, center_x, center_y, frame_width, frame_height)
                    
                    # Draw centering info on image
                    cv2.putText(img, "CENTERING MODE", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Estimate pose if calibration is available
                if calibration_loaded and camera_matrix is not None:
                    try:
                        ret = aruco.estimatePoseSingleMarkers(
                            corners[i:i+1], marker_size,
                            cameraMatrix=camera_matrix,
                            distCoeffs=dist_coeffs)
                        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                        
                        # Draw axis
                        try:
                            aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 10)
                        except AttributeError:
                            cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, 10)
                        
                        # Display position
                        position_text = "MARKER: x={:.2f} y={:.2f} z={:.2f} cm".format(
                            tvec[0], tvec[1], tvec[2])
                        cv2.putText(img, position_text, (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    except Exception as e:
                        logger.warning("Error estimating pose: {}".format(e))
        
        # Draw all detected markers
        try:
            aruco.drawDetectedMarkers(img, corners, ids)
        except:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
    
    return img, detection_data


def start_video_stream(client):
    """Start video stream using OpenCV VideoCapture."""
    global centering_mode, landing_mode
    logger = get_logger()
    logger.event("VIDEO_STREAM_START")

    # Create timestamp for video recording
    now = datetime.datetime.now()
    timestamp_string = now.strftime("%Y-%m-%d_%H-%M-%S")
    video_file = "./records/{}.mp4".format(timestamp_string)

    logger.info("Starting video stream")

    # Open camera
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    
    if not cap.isOpened():
        logger.error("Could not open camera")
        return
    
    logger.info("Camera opened successfully")
    
    # Video writer for recording
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(video_file, fourcc, 20.0, (frame_width, frame_height))
    
    frame_count = 0
    skip_frames = 2  # Skip frames for MQTT streaming
    aruco_skip_frames = 1  # Process every frame for ArUco
    
    logger.info("Starting video processing loop")
    
    while True:
        ret, img = cap.read()
        
        if not ret:
            logger.warning("Failed to read frame from camera")
            break
        
        frame_count += 1
        
        # Apply lens distortion correction if calibration is available
        if calibration_loaded and camera_matrix is not None and dist_coeffs is not None:
            img = cv2.undistort(img, camera_matrix, dist_coeffs)
        
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        try:
            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        except:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        # Process ArUco detections
        img, detection_data = process_aruco_detection(
            img, gray, corners, ids, frame_width, frame_height)
        
        # Publish ArUco detection data via MQTT
        detection_json = json.dumps(detection_data)
        client.publish(aruco_topic, detection_json, qos=0)
        
        # Publish image via ROS if available
        if image_pub is not None:
            try:
                # Convert BGR to RGB for ROS
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                ros_image = Image()
                ros_image.header.stamp = rospy.Time.now()
                ros_image.height = img_rgb.shape[0]
                ros_image.width = img_rgb.shape[1]
                ros_image.encoding = "rgb8"
                ros_image.step = img_rgb.shape[1] * 3
                ros_image.data = img_rgb.tobytes()
                image_pub.publish(ros_image)
            except Exception as e:
                logger.warning("Error publishing ROS image: {}".format(e))
        
        # Stream frame via MQTT (throttled)
        if frame_count % skip_frames == 0:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            ret_encode, jpeg = cv2.imencode(".jpg", img, encode_param)
            if ret_encode:
                jpg_as_text = base64.b64encode(jpeg).decode("utf-8")
                client.publish(stream_topic, jpg_as_text, qos=0)
        
        # Write frame to video file
        out.write(img)
        
        # Log metrics periodically
        if frame_count % 100 == 0:
            logger.metric("frames_processed", frame_count)
        
        # Display frame locally (optional, for debugging)
        # cv2.imshow("Drone Camera", img)
        
        # Break on 'q' key
        if cv2.waitKey(1) & 0xFF == ord("q"):
            logger.event("VIDEO_STREAM_STOP")
            logger.info("Video stream stopped by user")
            break
        
        # Allow ROS to process callbacks
        if not rospy.is_shutdown():
            rospy.sleep(0.001)
        else:
            break
    
    # Cleanup
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    logger.info("Video stream stopped and resources released")


def main():
    """Main function to start the drone client."""
    logger = init_logger()
    logger.event("SYSTEM_START")
    logger.info("Drone client starting (ROS simulation mode)")
    
    # Initialize ROS node
    try:
        rospy.init_node("drone_camera_node", anonymous=False)
        logger.info("ROS node initialized")
        
        # Create ROS publishers
        global image_pub, aruco_image_pub
        image_pub = rospy.Publisher("/camera/color/image_new", Image, queue_size=10)
        logger.info("ROS publishers created")
    except Exception as e:
        logger.warning("ROS not available: {}".format(e))
        logger.info("Running in MQTT-only mode")
    
    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        logger.info("Connecting to MQTT broker at {}:{}".format(
            broker_address, broker_port))
        client.connect(broker_address, broker_port, 60)
        logger.event("MQTT_CONNECTION_INITIATED")
    except Exception as e:
        logger.error("Could not connect to broker: {}".format(e))
        close_logger()
        return

    client.loop_start()
    logger.info("MQTT loop started")

    try:
        start_video_stream(client)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error("Video stream error: {}".format(e))
        import traceback
        traceback.print_exc()
    finally:
        client.loop_stop()
        client.disconnect()
        logger.event("SYSTEM_SHUTDOWN")
        logger.info("Drone client shutting down")
        close_logger()


if __name__ == "__main__":
    main()

