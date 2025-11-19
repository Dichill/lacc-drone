from threading import Thread, Lock
import paho.mqtt.client as mqtt
import json
import time
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
    b'LAND_SPEED', 20, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
response_topic = "drone/responses"
telemetry_topic = "drone/telemetry"
landing_target_topic = "drone/landing_target"

mqtt_client: Optional[mqtt.Client] = None
centering_mode: bool = False
landing_mode: bool = False
landing_mode_initialized: bool = False
takeoff_height: float = 4.0

smoothed_x_ang: float = 0.0
smoothed_y_ang: float = 0.0
last_command_time: float = 0.0
last_landing_command_time: float = 0.0
command_rate_limit: float = 0.2
landing_command_rate_limit: float = 0.05
last_manual_control_time: float = 0.0
manual_control_rate_limit: float = 0.05

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

latest_landing_target: Optional[Dict[str, Any]] = None
landing_target_lock: Lock = Lock()


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
    global landing_mode, centering_mode, current_armed, landing_mode_initialized
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
                    landing_mode_initialized = False
            
            was_armed = is_armed
            time.sleep(0.5)
            
        except Exception as e:
            logger.error(f"State monitor error: {e}", error=str(e))
            time.sleep(1)


def arm_and_takeoff(target_altitude: float) -> None:
    global landing_mode, centering_mode, current_armed, current_mode, landing_mode_initialized
    logger = get_logger()
    
    if landing_mode or centering_mode:
        logger.info("Resetting landing/centering modes from previous flight")
        landing_mode = False
        centering_mode = False
        landing_mode_initialized = False
    
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
    
    logger.info(f"Takeoff command sent to {target_altitude} meters", target_altitude=target_altitude)
    logger.event("TAKEOFF_COMMAND_SENT", altitude=target_altitude)
    
    time.sleep(2)
    logger.info("Takeoff complete - staying in GUIDED mode for manual control")
    logger.info("Note: Switch to LOITER manually if you want GPS position hold")


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


def landing_target_processor() -> None:
    global landing_mode, centering_mode, last_command_time, last_landing_command_time, latest_landing_target
    global landing_mode_initialized, smoothed_x_ang, smoothed_y_ang, current_altitude
    logger = get_logger()
    logger.info("Landing target processor started")
    
    while True:
        try:
            if landing_mode or centering_mode:
                with landing_target_lock:
                    target_data = latest_landing_target
                
                if target_data:
                    current_time = time.time()
                    x_ang = target_data.get("x_angle", 0.0)
                    y_ang = target_data.get("y_angle", 0.0)
                    
                    if landing_mode:
                        if not landing_mode_initialized:
                            smoothed_x_ang = x_ang
                            smoothed_y_ang = y_ang
                            landing_mode_initialized = True
                            logger.info("Landing mode initialized - smoothing filter reset")
                        
                        with telemetry_lock:
                            mode = current_mode
                            current_alt = current_altitude
                        
                        if mode != "LAND":
                            master.set_mode("LAND")
                            logger.info("Switching to LAND mode...")
                            logger.event("FLIGHT_MODE_CHANGED", mode="LAND")
                        
                        if current_alt > 2.0:
                            alpha = 0.5
                        elif current_alt > 1.0:
                            alpha = 0.6
                        else:
                            alpha = 0.7
                        
                        smoothed_x, smoothed_y = smooth_angular_position(x_ang, y_ang, alpha=alpha)
                        
                        if current_time - last_landing_command_time >= landing_command_rate_limit:
                            send_land_message(smoothed_x, smoothed_y)
                            last_landing_command_time = current_time
                            
                            if current_alt < 1.5:
                                angle_deg_x = abs(smoothed_x * 57.3)
                                angle_deg_y = abs(smoothed_y * 57.3)
                                logger.info(f"Landing: alt={current_alt:.1f}m, offset={angle_deg_x:.1f}deg, {angle_deg_y:.1f}deg",
                                          altitude=current_alt, offset_x=angle_deg_x, offset_y=angle_deg_y)
                    
                    elif centering_mode:
                        with telemetry_lock:
                            mode = current_mode
                        
                        if mode != "GUIDED":
                            logger.info(f"Centering mode requires GUIDED mode - switching from {mode}")
                            master.set_mode("GUIDED")
                            time.sleep(0.5)
                        
                        horizontal_res = 640
                        vertical_res = 480
                        
                        center_x = target_data.get("center_x", horizontal_res / 2)
                        center_y = target_data.get("center_y", vertical_res / 2)
                        
                        offset_x = (center_x - horizontal_res / 2) / (horizontal_res / 2)
                        offset_y = (center_y - vertical_res / 2) / (vertical_res / 2)
                        
                        smoothed_x, smoothed_y = smooth_angular_position(offset_x, offset_y, alpha=0.4)
                        
                        deadzone = 0.1
                        if abs(smoothed_x) < deadzone:
                            smoothed_x = 0.0
                        if abs(smoothed_y) < deadzone:
                            smoothed_y = 0.0
                        
                        velocity_gain = 0.3
                        vx = -smoothed_y * velocity_gain
                        vy = smoothed_x * velocity_gain
                        
                        if current_time - last_command_time >= command_rate_limit:
                            direction_msg = []
                            if abs(vx) > 0.01:
                                direction_msg.append("Forward" if vx > 0 else "Backward")
                            if abs(vy) > 0.01:
                                direction_msg.append("Right" if vy > 0 else "Left")
                            if direction_msg:
                                logger.info(f"Centering: {' + '.join(direction_msg)} (vx={vx:.2f}, vy={vy:.2f}, marker@{center_x:.0f},{center_y:.0f})",
                                          direction=" + ".join(direction_msg), vx=vx, vy=vy, marker_x=center_x, marker_y=center_y)
                            
                            send_local_ned_velocity(vx, vy, 0)
                            last_command_time = current_time
            
            time.sleep(0.05)
            
        except Exception as e:
            logger.error(f"Landing target processor error: {e}", error=str(e))
            time.sleep(0.1)


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
                global landing_mode_initialized
                centering_mode = False
                landing_mode = True
                landing_mode_initialized = False
                
                master.set_mode("LAND")
                logger.info("Switching to LAND mode - precision landing active")
                logger.event("FLIGHT_MODE_CHANGED", mode="LAND")
                logger.info("Auto-landing mode enabled - will guide to ArUco marker during descent")
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
    client.subscribe(landing_target_topic)
    logger.info(f"Subscribed to topic: {landing_target_topic}", topic=landing_target_topic)


def on_disconnect(client, userdata, rc: int) -> None:
    logger = get_logger()
    if rc != 0:
        logger.warning(f"Unexpected MQTT disconnect (code: {rc})", result_code=rc)
    else:
        logger.info("MQTT client disconnected")


def on_message(client, userdata, msg) -> None:
    global latest_landing_target
    logger = get_logger()
    
    if msg.topic == command_topic:
        payload: str = msg.payload.decode()
        logger.info(f"[{msg.topic}] {payload}", topic=msg.topic, payload=payload)
        
        command_thread: Thread = Thread(target=process_command, args=(payload,))
        command_thread.daemon = True
        command_thread.start()
    
    elif msg.topic == landing_target_topic:
        try:
            target_data = json.loads(msg.payload.decode())
            with landing_target_lock:
                latest_landing_target = target_data
        except Exception as e:
            logger.error(f"Failed to parse landing target: {e}", error=str(e))


def main() -> None:
    global mqtt_client
    logger = init_logger()
    
    logger.event("SYSTEM_START")
    logger.info("=" * 60)
    logger.info("LACC Drone - Flight Controller Client")
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
    
    landing_processor_thread: Thread = Thread(target=landing_target_processor)
    landing_processor_thread.daemon = True
    landing_processor_thread.start()
    logger.info("Landing target processor started")
    logger.info("")
    
    logger.info("Waiting for commands via MQTT...")
    logger.info(f"  - Send MQTT messages to topic: {command_topic}", topic=command_topic)
    logger.info("")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\n\nShutting down...")
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
