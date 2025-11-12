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

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from logger import get_logger, init_logger, close_logger

vehicle = connect("/dev/serial0", baud=57600, wait_ready=True)
vehicle.parameters["PLND_ENABLED"] = 1
vehicle.parameters["PLND_TYPE"] = 1
vehicle.parameters["PLND_EST_TYPE"] = 0
vehicle.parameters["LAND_SPEED"] = 30

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

last_stream_time: float = 0.0
stream_rate_limit: float = 0.05
last_aruco_publish_time: float = 0.0
aruco_publish_rate_limit: float = 0.2
last_detection_data: Dict[str, Any] = {"detected": False, "markers": [], "timestamp": time.time()}


def vehicle_state_monitor() -> None:
    global landing_mode, centering_mode
    logger = get_logger()
    logger.info("Vehicle state monitor started")
    
    was_armed: bool = False
    
    while True:
        try:
            is_armed: bool = vehicle.armed
            
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
    global landing_mode, centering_mode
    logger = get_logger()
    
    if landing_mode or centering_mode:
        logger.info("Resetting landing/centering modes from previous flight")
        landing_mode = False
        centering_mode = False
    
    while not vehicle.is_armable:
        logger.info("Waiting for vehicle to become armable")
        time.sleep(1)
    logger.info("Vehicle is now armable")
    logger.event("VEHICLE_ARMABLE")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode.name != "GUIDED":
        logger.info("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    logger.info("Vehicle now in GUIDED mode")
    logger.event("FLIGHT_MODE_CHANGED", mode="GUIDED")

    vehicle.armed = True
    while not vehicle.armed:
        logger.info("Waiting for vehicle to become armed")
        time.sleep(1)
    logger.info("Props are spinning!")
    logger.event("MOTORS_ARMED")

    vehicle.simple_takeoff(target_altitude)
    logger.event("TAKEOFF_INITIATED", altitude=target_altitude)

    start_time: float = time.time()
    timeout: int = 30
    while True:
        current_alt: float = vehicle.location.global_relative_frame.alt
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
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_land_message(x: float, y: float) -> None:
    msg = vehicle.message_factory.landing_target_encode(
        0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, x, y, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


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
                telemetry: Dict[str, Any] = {
                    "armed": vehicle.armed,
                    "mode": vehicle.mode.name,
                    "altitude": vehicle.location.global_relative_frame.alt,
                    "battery": vehicle.battery.level if vehicle.battery else None,
                    "gps_fix": vehicle.gps_0.fix_type if vehicle.gps_0 else None,
                    "landing_mode": landing_mode,
                    "centering_mode": centering_mode,
                    "ground_speed": vehicle.groundspeed if vehicle.groundspeed else 0.0,
                    "airspeed": vehicle.airspeed if vehicle.airspeed else 0.0,
                    "vertical_speed": vehicle.velocity[2] if vehicle.velocity and len(vehicle.velocity) > 2 else 0.0,
                    "heading": vehicle.heading if vehicle.heading else 0,
                    "latitude": vehicle.location.global_relative_frame.lat,
                    "longitude": vehicle.location.global_relative_frame.lon,
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
                if not vehicle.is_armable:
                    logger.warning("Vehicle not armable")
                    send_response(False, "Vehicle not armable", action)
                    return
                
                vehicle.mode = VehicleMode("GUIDED")
                while vehicle.mode.name != "GUIDED":
                    logger.info("Waiting for GUIDED mode...")
                    time.sleep(0.5)
                
                vehicle.armed = True
                timeout: int = 10
                start: float = time.time()
                while not vehicle.armed and (time.time() - start) < timeout:
                    logger.info("Waiting for vehicle to arm...")
                    time.sleep(0.5)
                
                if vehicle.armed:
                    logger.info("Vehicle armed successfully")
                    logger.event("MOTORS_ARMED")
                    send_response(True, "Vehicle armed", action)
                else:
                    logger.warning("Failed to arm vehicle")
                    send_response(False, "Arm timeout", action)
            except Exception as e:
                logger.error(f"Arm error: {e}", error=str(e))
                send_response(False, f"Arm failed: {str(e)}", action)

        elif action == "disarm":
            logger.event("DISARM_COMMAND")
            logger.info("Action: Executing 'disarm' command")
            try:
                centering_mode = False
                landing_mode = False
                
                current_mode: str = vehicle.mode.name
                if current_mode != "LAND" and vehicle.location.global_relative_frame.alt > 0.5:
                    logger.warning("Vehicle not landed. Switching to LAND mode first.")
                    vehicle.mode = VehicleMode("LAND")
                    send_response(True, "Landing before disarm", action)
                    return
                
                vehicle.armed = False
                timeout: int = 10
                start: float = time.time()
                while vehicle.armed and (time.time() - start) < timeout:
                    logger.info("Waiting for vehicle to disarm...")
                    time.sleep(0.5)
                
                if not vehicle.armed:
                    logger.info("Vehicle disarmed successfully")
                    logger.event("MOTORS_DISARMED")
                    send_response(True, "Vehicle disarmed", action)
                else:
                    logger.warning("Failed to disarm vehicle")
                    send_response(False, "Disarm timeout - vehicle may not be landed", action)
            except Exception as e:
                logger.error(f"Disarm error: {e}", error=str(e))
                send_response(False, f"Disarm failed: {str(e)}", action)

        elif action == "land":
            logger.event("LAND_COMMAND")
            logger.info("Action: Executing 'land' command")
            try:
                centering_mode = False
                landing_mode = False
                vehicle.mode = VehicleMode("LAND")
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
                    
                    current_alt: float = vehicle.location.global_relative_frame.alt
                    target_location: LocationGlobalRelative = LocationGlobalRelative(lat, lon, current_alt)
                    
                    logger.info(f"Moving to: lat={lat}, lon={lon}, alt={current_alt}", lat=lat, lon=lon, alt=current_alt)
                    vehicle.simple_goto(target_location)
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
                vehicle.mode = VehicleMode("LAND")
                
                def emergency_disarm():
                    time.sleep(5)
                    if vehicle.armed:
                        vehicle.armed = False
                        logger.warning("Emergency stop complete - vehicle disarmed")
                
                emergency_thread: Thread = Thread(target=emergency_disarm)
                emergency_thread.daemon = True
                emergency_thread.start()
                
                send_response(True, "Emergency stop activated", action)
            except Exception as e:
                logger.error(f"Emergency stop error: {e}", error=str(e))
                send_response(False, f"Emergency stop failed: {str(e)}", action)

        elif action == "manual_control":
            direction: str = data.get("direction", "")
            active: bool = data.get("active", False)
            
            try:
                if not vehicle.armed:
                    logger.warning("Manual control rejected - vehicle not armed")
                    send_response(False, "Vehicle not armed", action)
                    return
                
                if vehicle.mode.name != "GUIDED":
                    vehicle.mode = VehicleMode("GUIDED")
                    time.sleep(0.5)
                
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
                        msg = vehicle.message_factory.command_long_encode(
                            0, 0,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0,
                            0, -yaw_rate, 0, 1, 0, 0, 0
                        )
                        vehicle.send_mavlink(msg)
                        logger.info("Manual control: Yaw Left", direction=direction)
                    elif direction == "yaw-right":
                        msg = vehicle.message_factory.command_long_encode(
                            0, 0,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0,
                            0, yaw_rate, 0, 1, 0, 0, 0
                        )
                        vehicle.send_mavlink(msg)
                        logger.info("Manual control: Yaw Right", direction=direction)
                else:
                    send_local_ned_velocity(0, 0, 0)
                    logger.info(f"Manual control: Stop {direction}", direction=direction)
                    
            except Exception as e:
                logger.error(f"Manual control error: {e}", error=str(e))
                send_response(False, f"Manual control failed: {str(e)}", action)

        elif action == "status":
            logger.event("STATUS_REQUEST")
            logger.info("\n" + "=" * 50)
            logger.info("DRONE STATUS")
            logger.info("=" * 50)
            logger.info(f"  Armed: {vehicle.armed}", armed=vehicle.armed)
            logger.info(f"  Mode: {vehicle.mode.name}", mode=vehicle.mode.name)
            logger.info(f"  Altitude: {vehicle.location.global_relative_frame.alt:.2f}m", altitude=vehicle.location.global_relative_frame.alt)
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


def on_message(client, userdata, msg) -> None:
    logger = get_logger()
    payload: str = msg.payload.decode()
    logger.info(f"[{msg.topic}] {payload}", topic=msg.topic, payload=payload)
    process_command(payload)


def start_video_stream(client) -> None:
    global centering_mode, landing_mode, last_command_time
    global last_stream_time, last_aruco_publish_time, last_detection_data
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
        camera.configure(camera.create_video_configuration(main={"size": (horizontal_res, vertical_res)}))
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
        
        while True:
            with output3.condition:
                output3.condition.wait()
                frame = output3.frame

                if frame:
                    frame_count += 1
                    
                    img_array: np.ndarray = np.frombuffer(frame, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if calibration_loaded and camera_matrix is not None and dist_coeffs is not None:
                        img = cv2.undistort(img, camera_matrix, dist_coeffs)
                    
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                    
                    current_time: float = time.time()
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
                            if vehicle.mode.name != "LAND":
                                vehicle.mode = VehicleMode("LAND")
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
                    
                    should_stream: bool = (current_time - last_stream_time) >= stream_rate_limit
                    should_publish_aruco: bool = (current_time - last_aruco_publish_time) >= aruco_publish_rate_limit
                    
                    if should_publish_aruco:
                        detection_json: str = json.dumps(last_detection_data)
                        client.publish(aruco_topic, detection_json, qos=0)
                        last_aruco_publish_time = current_time
                    
                    if should_stream:
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 35]
                        ret, jpeg = cv2.imencode(".jpg", img, encode_param)
                        if ret:
                            jpg_as_text: str = base64.b64encode(jpeg).decode("utf-8")
                            client.publish(stream_topic, jpg_as_text, qos=0)
                        last_stream_time = current_time

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        logger.event("VIDEO_STREAM_STOP", frames_processed=frame_count)
                        logger.info(f"Video stream stopped - {frame_count} frames processed", frame_count=frame_count)
                        break

        output1.stop()
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
    mqtt_client.on_message = on_message

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
        close_logger()
    finally:
        print("Disconnected")
