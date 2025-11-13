import picamera2
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput, FfmpegOutput
from threading import Condition, Thread, Lock
import io

import base64
import datetime
import numpy as np
import paho.mqtt.client as mqtt
import cv2
import json
import time
import os
from typing import Optional, Dict, Any

from pymavlink import mavutil
from logger import get_logger, init_logger, close_logger

master = mavutil.mavlink_connection("/dev/serial0", baud=57600)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system} component {master.target_component}")

master.mav.param_set_send(
    master.target_system, master.target_component,
    b'PLND_ENABLED', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'PLND_TYPE', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'PLND_EST_TYPE', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'LAND_SPEED', 30, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
stream_topic = "camera/stream"
aruco_topic = "drone/aruco_detection"
response_topic = "drone/responses"
telemetry_topic = "drone/telemetry"

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
        print(f"⚠ No camera calibration found. Run 'python calibrate_camera.py' for better accuracy")
except Exception as e:
    print(f"⚠ Could not load calibration: {e}")

mqtt_client: Optional[mqtt.Client] = None
centering_mode: bool = False
landing_mode: bool = False
takeoff_height: float = 4.0

smoothed_x_ang: float = 0.0
smoothed_y_ang: float = 0.0
last_command_time: float = 0.0
command_rate_limit: float = 0.2
last_manual_control_time: float = 0.0
manual_control_rate_limit: float = 0.05

last_stream_time: float = 0.0
stream_rate_limit: float = 0.033
last_aruco_time: float = 0.0
aruco_rate_limit: float = 0.1
last_aruco_publish_time: float = 0.0
aruco_publish_rate_limit: float = 0.1
last_detection_data: Dict[str, Any] = {"detected": False, "markers": [], "timestamp": time.time()}

current_mode: str = "UNKNOWN"
current_armed: bool = False
current_altitude: float = 0.0
current_lat: float = 0.0
current_lon: float = 0.0
current_heading: int = 0
current_groundspeed: float = 0.0
current_airspeed: float = 0.0
current_vz: float = 0.0
telemetry_lock: Lock = Lock()


def get_mode_string(mode_num: int) -> str:
    mode_mapping = {
        0: "STABILIZE",
        1: "ACRO",
        2: "ALT_HOLD",
        3: "AUTO",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        7: "CIRCLE",
        9: "LAND",
        11: "DRIFT",
        13: "SPORT",
        14: "FLIP",
        15: "AUTOTUNE",
        16: "POSHOLD",
        17: "BRAKE",
        18: "THROW",
        19: "AVOID_ADSB",
        20: "GUIDED_NOGPS",
        21: "SMART_RTL",
        22: "FLOWHOLD",
        23: "FOLLOW",
        24: "ZIGZAG",
        25: "SYSTEMID",
        26: "AUTOROTATE",
        27: "AUTO_RTL"
    }
    return mode_mapping.get(mode_num, f"UNKNOWN({mode_num})")


def telemetry_listener() -> None:
    global current_mode, current_armed, current_altitude, current_lat, current_lon
    global current_heading, current_groundspeed, current_airspeed, current_vz
    logger = get_logger()
    logger.info("Telemetry listener started")
    
    while True:
        try:
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            
            msg_type = msg.get_type()
            
            with telemetry_lock:
                if msg_type == "HEARTBEAT":
                    current_mode = get_mode_string(msg.custom_mode)
                    current_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    
                elif msg_type == "GLOBAL_POSITION_INT":
                    current_altitude = msg.relative_alt / 1000.0
                    current_lat = msg.lat / 1e7
                    current_lon = msg.lon / 1e7
                    current_heading = msg.hdg // 100
                    current_vz = msg.vz / 100.0
                    
                elif msg_type == "VFR_HUD":
                    current_groundspeed = msg.groundspeed
                    current_airspeed = msg.airspeed
                    
        except Exception as e:
            logger.error(f"Telemetry listener error: {e}", error=str(e))
            time.sleep(0.1)


def vehicle_state_monitor() -> None:
    global landing_mode, centering_mode, current_armed
    logger = get_logger()
    logger.info("Vehicle state monitor started")
    
    was_armed: bool = False
    
    while True:
        try:
            with telemetry_lock:
                is_armed: bool = current_armed
            
            if was_armed and not is_armed:
                if landing_mode or centering_mode:
                    logger.info("Vehicle disarmed - auto-disabling landing/centering modes")
                    landing_mode = False
                    centering_mode = False
            
            was_armed = is_armed
            time.sleep(0.5)
            
        except Exception as e:
            logger.error(f"State monitor error: {e}", error=str(e))
            time.sleep(1)


def arm_and_takeoff(target_altitude: float) -> None:
    global landing_mode, centering_mode, current_armed, current_mode, current_altitude
    logger = get_logger()
    
    if landing_mode or centering_mode:
        logger.info("Resetting landing/centering modes from previous flight")
        landing_mode = False
        centering_mode = False
    
    with telemetry_lock:
        if current_armed:
            logger.info("Vehicle is already armed")
        else:
            logger.info("Arming motors...")
            logger.event("ARMING_MOTORS")
            master.arducopter_arm()
            time.sleep(2)
            logger.event("MOTORS_ARMED")
    
    logger.info("Setting flight mode to GUIDED...")
    master.set_mode("GUIDED")
    logger.event("FLIGHT_MODE_CHANGED", mode="GUIDED")
    time.sleep(1)
    
    logger.info(f"Taking off to {target_altitude} meters...", target_altitude=target_altitude)
    logger.event("TAKEOFF_INITIATED", altitude=target_altitude)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )
    
    start_time: float = time.time()
    timeout: int = 30
    while True:
        with telemetry_lock:
            current_alt: float = current_altitude
        
        logger.info(f"Current Altitude: {current_alt:.2f}m", altitude=current_alt)
        
        if current_alt >= 0.95 * target_altitude:
            logger.info("Target altitude reached!")
            logger.event("TARGET_ALTITUDE_REACHED", target=target_altitude, current=current_alt)
            break
        
        if time.time() - start_time > timeout:
            logger.warning(f"Altitude check timed out after {timeout} seconds", timeout=timeout)
            logger.warning(f"Current altitude: {current_alt} (target: {target_altitude})", current=current_alt, target=target_altitude)
            break
        
        time.sleep(1)


def send_local_ned_velocity(vx: float, vy: float, vz: float) -> None:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


def send_land_message(x: float, y: float) -> None:
    master.mav.landing_target_send(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x, y, 0, 0, 0
    )


def smooth_angular_position(x_ang: float, y_ang: float, alpha: float = 0.3) -> tuple:
    global smoothed_x_ang, smoothed_y_ang
    
    smoothed_x_ang = alpha * x_ang + (1.0 - alpha) * smoothed_x_ang
    smoothed_y_ang = alpha * y_ang + (1.0 - alpha) * smoothed_y_ang
    
    return smoothed_x_ang, smoothed_y_ang


def send_response(success: bool, message: str, action: str) -> None:
    global mqtt_client
    logger = get_logger()
    if mqtt_client:
        try:
            response: Dict[str, Any] = {
                "success": success,
                "message": message,
                "action": action,
                "timestamp": time.time()
            }
            mqtt_client.publish(response_topic, json.dumps(response), qos=1)
        except Exception as e:
            logger.error(f"Failed to send response: {e}", error=str(e))


def telemetry_publisher() -> None:
    global mqtt_client
    logger = get_logger()
    
    while True:
        try:
            if mqtt_client:
                with telemetry_lock:
                    telemetry: Dict[str, Any] = {
                        "armed": current_armed,
                        "mode": current_mode,
                        "altitude": current_altitude,
                        "battery": None,
                        "gps_fix": None,
                        "landing_mode": landing_mode,
                        "centering_mode": centering_mode,
                        "ground_speed": current_groundspeed,
                        "airspeed": current_airspeed,
                        "vertical_speed": current_vz,
                        "heading": current_heading,
                        "latitude": current_lat,
                        "longitude": current_lon,
                        "timestamp": time.time()
                    }
                mqtt_client.publish(telemetry_topic, json.dumps(telemetry), qos=0)
        except Exception as e:
            logger.error(f"Telemetry error: {e}", error=str(e))
        
        time.sleep(1)


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def process_command(command: str) -> None:
    global centering_mode, landing_mode
    logger = get_logger()
    logger.info(f"Processing command: {command}", command=command)

    try:
        data: Dict[str, Any] = json.loads(command)
        action: str = data.get("action", "")
        value: str = data.get("value", "")

        if action == "takeoff":
            logger.event("TAKEOFF_COMMAND")
            logger.info("Action: Executing 'takeoff' command")
            target_altitude: float = float(data.get("altitude", takeoff_height))
            logger.info(f"Target altitude set to {target_altitude} meters", altitude=target_altitude)
            
            def takeoff_thread():
                try:
                    arm_and_takeoff(target_altitude)
                    logger.info("Takeoff complete")
                    send_response(True, f"Takeoff completed at {target_altitude} meters", action)
                except Exception as e:
                    logger.error(f"Takeoff failed: {e}", error=str(e))
                    send_response(False, f"Takeoff failed: {str(e)}", action)
            
            t: Thread = Thread(target=takeoff_thread)
            t.daemon = True
            t.start()
            logger.info("Takeoff initiated (running in background)")
            send_response(True, "Takeoff command received", action)

        elif action == "arm":
            logger.event("ARM_COMMAND")
            logger.info("Action: Executing 'arm' command")
            try:
                logger.info("Setting flight mode to GUIDED...")
                master.set_mode("GUIDED")
                time.sleep(1)
                
                logger.info("Arming motors...")
                master.arducopter_arm()
                time.sleep(2)
                
                logger.info("Vehicle armed successfully")
                logger.event("MOTORS_ARMED")
                send_response(True, "Vehicle armed", action)
            except Exception as e:
                logger.error(f"Arm error: {e}", error=str(e))
                send_response(False, f"Arm failed: {str(e)}", action)

        elif action == "disarm":
            logger.event("DISARM_COMMAND")
            logger.info("Action: Executing 'disarm' command")
            try:
                centering_mode = False
                landing_mode = False
                
                with telemetry_lock:
                    mode = current_mode
                    alt = current_altitude
                
                if mode != "LAND" and alt > 0.5:
                    logger.warning("Vehicle not landed. Switching to LAND mode first.")
                    master.set_mode("LAND")
                    send_response(True, "Landing before disarm", action)
                    return
                
                logger.info("Disarming motors...")
                master.arducopter_disarm()
                time.sleep(1)
                
                logger.info("Vehicle disarmed successfully")
                logger.event("MOTORS_DISARMED")
                send_response(True, "Vehicle disarmed", action)
            except Exception as e:
                logger.error(f"Disarm error: {e}", error=str(e))
                send_response(False, f"Disarm failed: {str(e)}", action)

        elif action == "land":
            logger.event("LAND_COMMAND")
            logger.info("Action: Executing 'land' command")
            try:
                centering_mode = False
                landing_mode = False
                master.set_mode("LAND")
                logger.info("Regular landing initiated")
                logger.event("FLIGHT_MODE_CHANGED", mode="LAND")
                send_response(True, "Landing initiated", action)
            except Exception as e:
                logger.error(f"Land error: {e}", error=str(e))
                send_response(False, f"Land failed: {str(e)}", action)

        elif action == "auto_land":
            logger.event("AUTO_LAND_COMMAND")
            logger.info("Action: Executing 'auto_land' command")
            try:
                centering_mode = False
                landing_mode = True
                logger.info("Auto-landing mode enabled")
                send_response(True, "Auto-landing enabled", action)
            except Exception as e:
                logger.error(f"Auto-land error: {e}", error=str(e))
                send_response(False, f"Auto-land failed: {str(e)}", action)

        elif action == "enable_centering":
            logger.event("CENTERING_MODE_ENABLED")
            try:
                centering_mode = True
                logger.info("Centering mode enabled")
                send_response(True, "Centering enabled", action)
            except Exception as e:
                logger.error(f"Enable centering error: {e}", error=str(e))
                send_response(False, f"Enable centering failed: {str(e)}", action)

        elif action == "disable_centering":
            logger.event("CENTERING_MODE_DISABLED")
            try:
                centering_mode = False
                logger.info("Centering mode disabled")
                send_response(True, "Centering disabled", action)
            except Exception as e:
                logger.error(f"Disable centering error: {e}", error=str(e))
                send_response(False, f"Disable centering failed: {str(e)}", action)

        elif action == "move":
            logger.event("MOVE_COMMAND", destination=value)
            logger.info(f"Action: Executing 'move' command to: {value}")
            try:
                coords: list = value.split(",")
                if len(coords) == 2:
                    lat: float = float(coords[0].strip())
                    lon: float = float(coords[1].strip())
                    
                    with telemetry_lock:
                        alt: float = current_altitude
                    
                    logger.info(f"Moving to: lat={lat}, lon={lon}, alt={alt}", lat=lat, lon=lon, alt=alt)
                    master.mav.mission_item_send(
                        master.target_system,
                        master.target_component,
                        0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        2, 0, 0, 0, 0, 0,
                        lat, lon, alt
                    )
                    send_response(True, f"Navigation to ({lat}, {lon}) initiated", action)
                else:
                    logger.warning("Invalid coordinate format")
                    send_response(False, "Invalid format. Expected 'lat,lon'", action)
            except (ValueError, AttributeError) as e:
                logger.error(f"Move error: {e}", error=str(e))
                send_response(False, f"Move failed: {str(e)}", action)

        elif action == "emergency_stop":
            logger.event("EMERGENCY_STOP")
            logger.critical("!!! EMERGENCY STOP ACTIVATED !!!")
            try:
                centering_mode = False
                landing_mode = False
                master.set_mode("LAND")
                
                def emergency_disarm():
                    time.sleep(5)
                    master.arducopter_disarm()
                    logger.warning("Emergency stop complete - vehicle disarmed")
                
                emergency_thread: Thread = Thread(target=emergency_disarm)
                emergency_thread.daemon = True
                emergency_thread.start()
                
                send_response(True, "Emergency stop activated", action)
            except Exception as e:
                logger.error(f"Emergency stop error: {e}", error=str(e))
                send_response(False, f"Emergency stop failed: {str(e)}", action)

        elif action == "manual_control":
            global last_manual_control_time
            direction: str = data.get("direction", "")
            active: bool = data.get("active", False)
            
            current_time: float = time.time()
            if current_time - last_manual_control_time < manual_control_rate_limit:
                return
            last_manual_control_time = current_time
            
            try:
                with telemetry_lock:
                    armed = current_armed
                    mode = current_mode
                
                if not armed:
                    logger.warning("Manual control rejected - vehicle not armed")
                    return
                
                if mode != "GUIDED":
                    master.set_mode("GUIDED")
                    time.sleep(0.3)
                
                velocity_scale: float = 2.0
                yaw_rate: float = 30.0
                
                if active:
                    if direction == "forward":
                        send_local_ned_velocity(velocity_scale, 0, 0)
                        logger.info("Manual control: Forward", direction=direction)
                    elif direction == "backward":
                        send_local_ned_velocity(-velocity_scale, 0, 0)
                        logger.info("Manual control: Backward", direction=direction)
                    elif direction == "left":
                        send_local_ned_velocity(0, -velocity_scale, 0)
                        logger.info("Manual control: Left", direction=direction)
                    elif direction == "right":
                        send_local_ned_velocity(0, velocity_scale, 0)
                        logger.info("Manual control: Right", direction=direction)
                    elif direction == "up":
                        send_local_ned_velocity(0, 0, -velocity_scale)
                        logger.info("Manual control: Up", direction=direction)
                    elif direction == "down":
                        send_local_ned_velocity(0, 0, velocity_scale)
                        logger.info("Manual control: Down", direction=direction)
                    elif direction == "yaw-left":
                        master.mav.command_long_send(
                            master.target_system,
                            master.target_component,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0, 0, -yaw_rate, 0, 1, 0, 0, 0
                        )
                        logger.info("Manual control: Yaw Left", direction=direction)
                    elif direction == "yaw-right":
                        master.mav.command_long_send(
                            master.target_system,
                            master.target_component,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0, 0, yaw_rate, 0, 1, 0, 0, 0
                        )
                        logger.info("Manual control: Yaw Right", direction=direction)
                else:
                    send_local_ned_velocity(0, 0, 0)
                    logger.info(f"Manual control: Stop {direction}", direction=direction)
                    
            except Exception as e:
                logger.error(f"Manual control error: {e}", error=str(e))

        elif action == "status":
            logger.event("STATUS_REQUEST")
            logger.info("\n" + "=" * 50)
            logger.info("DRONE STATUS")
            logger.info("=" * 50)
            with telemetry_lock:
                logger.info(f"  Armed: {current_armed}", armed=current_armed)
                logger.info(f"  Mode: {current_mode}", mode=current_mode)
                logger.info(f"  Altitude: {current_altitude:.2f}m", altitude=current_altitude)
            logger.info(f"  Landing Mode: {'ACTIVE' if landing_mode else 'INACTIVE'}", landing_mode=landing_mode)
            logger.info(f"  Centering Mode: {'ACTIVE' if centering_mode else 'INACTIVE'}", centering_mode=centering_mode)
            logger.info("=" * 50 + "\n")

        else:
            logger.warning(f"Unknown command action: {action}", action=action)
            send_response(False, f"Unknown command: {action}", action)

    except (json.JSONDecodeError, ValueError) as e:
        logger.warning(f"Received invalid message: {e}", error=str(e))
        send_response(False, "Invalid message format", "unknown")


def on_connect(client, userdata, flags, rc: int) -> None:
    logger = get_logger()
    logger.event("MQTT_CONNECTED", result_code=rc)
    logger.info(f"Connected to MQTT Broker with result code {rc}", result_code=rc)
    client.subscribe(command_topic)
    logger.info(f"Subscribed to topic: {command_topic}", topic=command_topic)


def on_disconnect(client, userdata, rc: int) -> None:
    logger = get_logger()
    if rc != 0:
        logger.warning(f"Unexpected MQTT disconnect (code: {rc})", result_code=rc)
    else:
        logger.info("MQTT client disconnected")


def on_message(client, userdata, msg) -> None:
    logger = get_logger()
    payload: str = msg.payload.decode()
    logger.info(f"[{msg.topic}] {payload}", topic=msg.topic, payload=payload)
    
    command_thread: Thread = Thread(target=process_command, args=(payload,))
    command_thread.daemon = True
    command_thread.start()


def start_video_stream(client) -> None:
    global centering_mode, landing_mode, last_command_time
    global last_stream_time, last_aruco_time, last_aruco_publish_time, last_detection_data
    logger = get_logger()

    now: datetime.datetime = datetime.datetime.now()
    timestamp_string: str = now.strftime("%Y-%m-%d_%H-%M-%S")
    video_file: str = f"./records/{timestamp_string}.mp4"

    logger.event("VIDEO_STREAM_START")
    logger.info(f"Starting video stream - recording to {video_file}", video_file=video_file)

    marker_size: float = 20.0
    horizontal_res: int = 640
    vertical_res: int = 480
    horizontal_fov: float = 62.2 * (np.pi / 180)
    vertical_fov: float = 48.8 * (np.pi / 180)

    with Picamera2() as camera:
        config = camera.create_video_configuration(
            main={"size": (horizontal_res, vertical_res)},
            controls={
                "AfMode": 0,
                "FrameRate": 30.0
            }
        )
        camera.configure(config)
        
        encoder: JpegEncoder = JpegEncoder()
        output1: FfmpegOutput = FfmpegOutput(video_file, audio=False)
        output3: StreamingOutput = StreamingOutput()
        output2: FileOutput = FileOutput(output3)
        encoder.output = [output1, output2]

        camera.start_encoder(encoder)
        camera.start()
        output1.start()
        logger.info("Camera encoder started")
        
        try:
            cv2.namedWindow("Drone Camera Feed", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Drone Camera Feed", 640, 480)
            logger.info("Display window created")
        except Exception as e:
            logger.warning(f"Could not create display window (running headless?): {e}")

        frame_count: int = 0
        
        while True:
            with output3.condition:
                output3.condition.wait()
                frame = output3.frame

                if frame:
                    frame_count += 1
                    current_time: float = time.time()
                    
                    should_process_aruco: bool = (current_time - last_aruco_time) >= aruco_rate_limit
                    force_process: bool = landing_mode or centering_mode
                    should_stream: bool = (current_time - last_stream_time) >= stream_rate_limit
                    
                    img_array: np.ndarray = np.frombuffer(frame, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if calibration_loaded and camera_matrix is not None and dist_coeffs is not None:
                        img = cv2.undistort(img, camera_matrix, dist_coeffs)
                    
                    if should_process_aruco or force_process:
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
                        
                        if landing_mode:
                            with telemetry_lock:
                                mode = current_mode
                            if mode != "LAND":
                                master.set_mode("LAND")
                                logger.info("Switching to LAND mode...")
                                logger.event("FLIGHT_MODE_CHANGED", mode="LAND")
                            
                            smoothed_x, smoothed_y = smooth_angular_position(x_ang, y_ang, alpha=0.2)
                            
                            if current_time - last_command_time >= command_rate_limit:
                                send_land_message(smoothed_x, smoothed_y)
                                last_command_time = current_time
                        
                        elif centering_mode:
                            offset_x: float = (x_avg - horizontal_res / 2) / (horizontal_res / 2)
                            offset_y: float = (y_avg - vertical_res / 2) / (vertical_res / 2)
                            
                            smoothed_x, smoothed_y = smooth_angular_position(offset_x, offset_y, alpha=0.4)
                            
                            deadzone: float = 0.1
                            if abs(smoothed_x) < deadzone:
                                smoothed_x = 0.0
                            if abs(smoothed_y) < deadzone:
                                smoothed_y = 0.0
                            
                            velocity_gain: float = 0.3
                            vx: float = smoothed_y * velocity_gain
                            vy: float = smoothed_x * velocity_gain
                            
                            if current_time - last_command_time >= command_rate_limit:
                                send_local_ned_velocity(vx, vy, 0)
                                last_command_time = current_time
                        
                        cv2.aruco.drawDetectedMarkers(img, corners, ids)
                        
                        if landing_mode:
                            cv2.putText(img, "LANDING MODE", (10, 30), 0, 0.6, (0, 0, 255), thickness=2)
                        elif centering_mode:
                            cv2.putText(img, "CENTERING MODE", (10, 30), 0, 0.6, (0, 255, 0), thickness=2)
                        
                        detection_data = {
                            "detected": True,
                            "marker_id": marker_id,
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
                                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
                                ret, jpeg = cv2.imencode(".jpg", img, encode_param)
                                if ret:
                                    jpg_as_text: str = base64.b64encode(jpeg).decode("utf-8")
                                    client.publish(stream_topic, jpg_as_text, qos=0)
                                last_stream_time = current_time
                    except Exception as e:
                        logger.error(f"MQTT publish error: {e}", error=str(e))
                    
                    with telemetry_lock:
                        mode = current_mode
                        armed = current_armed
                    
                    status_text = f"Mode: {mode} | Armed: {armed} | Frames: {frame_count}"
                    cv2.putText(img, status_text, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    try:
                        cv2.imshow("Drone Camera Feed", img)
                    except Exception as e:
                        logger.warning(f"Display error (running headless?): {e}")

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        logger.event("VIDEO_STREAM_STOP", frames_processed=frame_count)
                        logger.info(f"Video stream stopped - {frame_count} frames processed", frame_count=frame_count)
                        break

        output1.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        logger.info("Video recording stopped")


def main() -> None:
    global mqtt_client
    logger = init_logger()
    
    logger.event("SYSTEM_START")
    logger.info("=" * 60)
    logger.info("LACC Drone - Real Hardware Client")
    logger.info("=" * 60)
    logger.info("\nConfiguration:")
    logger.info("  Vehicle: /dev/serial0 @ 57600 baud")
    logger.info(f"  MQTT Broker: {broker_address}:{broker_port}", broker=broker_address, port=broker_port)
    logger.info(f"  Default Takeoff Height: {takeoff_height}m", takeoff_height=takeoff_height)
    logger.info("=" * 60)
    logger.info("")

    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message
    mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)

    try:
        logger.info(f"Connecting to MQTT broker at {broker_address}:{broker_port}...", broker=broker_address, port=broker_port)
        mqtt_client.connect(broker_address, broker_port, 60)
        mqtt_client.loop_start()
        logger.event("MQTT_CONNECTION_INITIATED")
        logger.info("MQTT connected and ready for commands")
        logger.info("")
    except Exception as e:
        logger.error(f"Could not connect to broker: {e}", error=str(e), error_type=type(e).__name__)
        close_logger()
        return

    telemetry_listener_thread: Thread = Thread(target=telemetry_listener)
    telemetry_listener_thread.daemon = True
    telemetry_listener_thread.start()
    logger.info("Telemetry listener started")
    
    monitor_thread: Thread = Thread(target=vehicle_state_monitor)
    monitor_thread.daemon = True
    monitor_thread.start()
    logger.info("Vehicle state monitor started")
    
    telemetry_thread: Thread = Thread(target=telemetry_publisher)
    telemetry_thread.daemon = True
    telemetry_thread.start()
    logger.info("Telemetry publisher started")
    logger.info("")
    
    logger.info("Waiting for commands via MQTT...")
    logger.info(f"  - Send MQTT messages to topic: {command_topic}", topic=command_topic)
    logger.info("")

    try:
        start_video_stream(mqtt_client)
    except Exception as e:
        logger.error(f"Video stream error: {e}", error=str(e), error_type=type(e).__name__)
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        logger.event("SYSTEM_SHUTDOWN")
        logger.info("Drone client shutting down")
        close_logger()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger = get_logger()
        logger.info("\n\nShutting down...")
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        close_logger()
    finally:
        print("Disconnected")
