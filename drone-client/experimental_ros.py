#!/usr/bin/env python

import time
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
from threading import Thread, Lock

mav = mavutil.mavlink_connection("tcp:127.0.0.1:5763")
mav.wait_heartbeat()
print("Heartbeat received (system {}, component {})".format(mav.target_system, mav.target_component))

print("Requesting data streams...")
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4,
    1
)
print("Data streams requested at 4 Hz")

velocity = -0.5
takeoff_height = 4

broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
response_topic = "drone/responses"
telemetry_topic = "drone/telemetry"
landing_target_topic = "drone/landing_target"

mqtt_client = None
centering_mode = False
landing_mode = False
landing_mode_initialized = False

smoothed_x_ang = 0.0
smoothed_y_ang = 0.0
last_command_time = 0.0
last_landing_command_time = 0.0
command_rate_limit = 0.2
landing_command_rate_limit = 0.05

latest_landing_target = None
landing_target_lock = Lock()

vehicle_state = {
    "armed": False,
    "mode": "UNKNOWN",
    "mode_num": 0,
    "altitude": 0.0,
    "battery_voltage": 0.0,
    "battery_level": None,
    "gps_fix": 0,
    "ground_speed": 0.0,
    "airspeed": 0.0,
    "vertical_speed": 0.0,
    "heading": 0,
    "latitude": 0.0,
    "longitude": 0.0,
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
}
state_lock = Lock()


def set_parameter(param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        param_name.encode("utf8"),
        param_value,
        param_type
    )


def get_mode_name(mode_num):
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
    }
    return mode_mapping.get(mode_num, "UNKNOWN")


def set_mode(mode_name):
    mav.set_mode(mode_name)


def arm_vehicle():
    mav.arducopter_arm()


def disarm_vehicle():
    mav.arducopter_disarm()


def message_listener():
    global vehicle_state
    
    message_count = 0
    last_report = time.time()
    
    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if not msg:
                continue
            
            msg_type = msg.get_type()
            message_count += 1
            
            if time.time() - last_report > 10:
                print("Message listener active: {} messages received".format(message_count))
                last_report = time.time()
            
            if msg_type == "HEARTBEAT":
                with state_lock:
                    vehicle_state["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    vehicle_state["mode_num"] = msg.custom_mode
                    vehicle_state["mode"] = get_mode_name(msg.custom_mode)
            
            elif msg_type == "GLOBAL_POSITION_INT":
                with state_lock:
                    vehicle_state["latitude"] = msg.lat / 1e7
                    vehicle_state["longitude"] = msg.lon / 1e7
                    vehicle_state["altitude"] = msg.relative_alt / 1000.0
                    vehicle_state["heading"] = msg.hdg // 100
                    vehicle_state["vx"] = msg.vx / 100.0
                    vehicle_state["vy"] = msg.vy / 100.0
                    vehicle_state["vz"] = msg.vz / 100.0
            
            elif msg_type == "VFR_HUD":
                with state_lock:
                    vehicle_state["ground_speed"] = msg.groundspeed
                    vehicle_state["airspeed"] = msg.airspeed
                    vehicle_state["vertical_speed"] = msg.climb
            
            elif msg_type == "SYS_STATUS":
                with state_lock:
                    vehicle_state["battery_voltage"] = msg.voltage_battery / 1000.0
                    if msg.battery_remaining >= 0:
                        vehicle_state["battery_level"] = msg.battery_remaining
            
            elif msg_type == "GPS_RAW_INT":
                with state_lock:
                    vehicle_state["gps_fix"] = msg.fix_type
        
        except Exception as e:
            print("Message listener error: {}".format(e))
            time.sleep(0.1)


def vehicle_state_monitor():
    global landing_mode, centering_mode, landing_mode_initialized, latest_landing_target
    print("Vehicle state monitor started")
    
    was_armed = False
    
    while True:
        try:
            with state_lock:
                is_armed = vehicle_state["armed"]
            
            if was_armed and not is_armed:
                if landing_mode or centering_mode:
                    print("Vehicle disarmed - auto-disabling landing/centering modes")
                    landing_mode = False
                    centering_mode = False
                    landing_mode_initialized = False
                    
                    with landing_target_lock:
                        latest_landing_target = None
                    print("Cleared stale landing target data")
            
            was_armed = is_armed
            time.sleep(0.5)
            
        except Exception as e:
            print("State monitor error: {}".format(e))
            time.sleep(1)


def arm_and_takeoff(targetHeight):
    global landing_mode, centering_mode, landing_mode_initialized, latest_landing_target
    
    if landing_mode or centering_mode:
        print("Resetting landing/centering modes from previous flight")
        landing_mode = False
        centering_mode = False
        landing_mode_initialized = False
        
        with landing_target_lock:
            latest_landing_target = None
        print("Cleared stale landing target data")
    
    with state_lock:
        if vehicle_state["armed"]:
            print("Vehicle is already armed")
        else:
            print("Arming motors...")
            arm_vehicle()
            time.sleep(2)
    
    print("Setting flight mode to GUIDED...")
    set_mode("GUIDED")
    time.sleep(1)
    
    print("Vehicle now in GUIDED mode. Have Fun!")
    
    print("Taking off to {} meters...".format(targetHeight))
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, targetHeight
    )
    
    print("Takeoff command sent to {} meters".format(targetHeight))
    
    time.sleep(2)
    print("Takeoff complete - staying in GUIDED mode for manual control")
    print("Note: Switch to LOITER manually if you want GPS position hold")

    return None


def send_local_ned_velocity(vx, vy, vz):
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


def send_land_message(x, y):
    mav.mav.landing_target_send(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x, y,
        0, 0, 0
    )


def smooth_angular_position(x_ang, y_ang, alpha=0.3):
    global smoothed_x_ang, smoothed_y_ang
    
    smoothed_x_ang = alpha * x_ang + (1.0 - alpha) * smoothed_x_ang
    smoothed_y_ang = alpha * y_ang + (1.0 - alpha) * smoothed_y_ang
    
    return smoothed_x_ang, smoothed_y_ang


def send_response(success, message, action):
    global mqtt_client
    if mqtt_client:
        try:
            response = {
                "success": success,
                "message": message,
                "action": action,
                "timestamp": time.time()
            }
            mqtt_client.publish(response_topic, json.dumps(response), qos=1)
        except Exception as e:
            print("Failed to send response: {}".format(e))


def telemetry_publisher():
    global mqtt_client
    
    while True:
        try:
            if mqtt_client:
                with state_lock:
                    telemetry = {
                        "armed": vehicle_state["armed"],
                        "mode": vehicle_state["mode"],
                        "altitude": vehicle_state["altitude"],
                        "battery": vehicle_state["battery_level"],
                        "gps_fix": vehicle_state["gps_fix"],
                        "landing_mode": landing_mode,
                        "centering_mode": centering_mode,
                        "ground_speed": vehicle_state["ground_speed"],
                        "airspeed": vehicle_state["airspeed"],
                        "vertical_speed": vehicle_state["vertical_speed"],
                        "heading": vehicle_state["heading"],
                        "latitude": vehicle_state["latitude"],
                        "longitude": vehicle_state["longitude"],
                        "timestamp": time.time()
                    }
                mqtt_client.publish(telemetry_topic, json.dumps(telemetry), qos=0)
        except Exception as e:
            print("Telemetry error: {}".format(e))
        
        time.sleep(1)


def landing_target_processor():
    global landing_mode, centering_mode, last_command_time, last_landing_command_time, latest_landing_target
    global landing_mode_initialized, smoothed_x_ang, smoothed_y_ang
    print("Landing target processor started")
    
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
                            print("Landing mode initialized - smoothing filter reset")
                        
                        with state_lock:
                            current_mode = vehicle_state["mode"]
                        
                        if current_mode != "LAND":
                            set_mode("LAND")
                            print("Switching to LAND mode...")
                        
                        with state_lock:
                            current_alt = vehicle_state["altitude"]
                        
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
                                print("Landing: alt={:.1f}m, offset={:.1f}deg, {:.1f}deg".format(
                                    current_alt, angle_deg_x, angle_deg_y))
                    
                    elif centering_mode:
                        with state_lock:
                            current_mode = vehicle_state["mode"]
                            current_alt = vehicle_state["altitude"]
                        
                        if current_mode != "GUIDED":
                            print("Centering mode requires GUIDED mode - switching from {}".format(current_mode))
                            set_mode("GUIDED")
                            time.sleep(0.5)
                        
                        horizontal_res = 640
                        vertical_res = 480
                        
                        center_x = target_data.get("center_x", horizontal_res / 2)
                        center_y = target_data.get("center_y", vertical_res / 2)
                        
                        offset_x = (center_x - horizontal_res / 2) / (horizontal_res / 2)
                        offset_y = (center_y - vertical_res / 2) / (vertical_res / 2)
                        
                        smoothed_x, smoothed_y = smooth_angular_position(offset_x, offset_y, alpha=0.4)
                        
                        if current_alt < 2.0:
                            deadzone = 0.08
                            velocity_gain = 0.35
                        elif current_alt < 5.0:
                            deadzone = 0.1
                            velocity_gain = 0.25
                        elif current_alt < 8.0:
                            deadzone = 0.12
                            velocity_gain = 0.15
                        else:
                            deadzone = 0.15
                            velocity_gain = 0.10
                        
                        if abs(smoothed_x) < deadzone:
                            smoothed_x = 0.0
                        if abs(smoothed_y) < deadzone:
                            smoothed_y = 0.0
                        
                        vx = -smoothed_y * velocity_gain
                        vy = smoothed_x * velocity_gain
                        
                        if current_time - last_command_time >= command_rate_limit:
                            direction_msg = []
                            if abs(vx) > 0.01:
                                direction_msg.append("Forward" if vx > 0 else "Backward")
                            if abs(vy) > 0.01:
                                direction_msg.append("Right" if vy > 0 else "Left")
                            
                            offset_info = "offset_x={:.2f} offset_y={:.2f}".format(offset_x, offset_y)
                            position_info = "marker@({:.0f},{:.0f}) image_center@(320,240)".format(center_x, center_y)
                            
                            if direction_msg or abs(vx) > 0.01 or abs(vy) > 0.01:
                                print("Centering: {} | alt={:.1f}m gain={:.2f} | vx={:.2f} vy={:.2f} | {} | {}".format(
                                    " + ".join(direction_msg) if direction_msg else "Holding", 
                                    current_alt, velocity_gain, vx, vy, offset_info, position_info))
                            
                            send_local_ned_velocity(vx, vy, 0)
                            last_command_time = current_time
            
            time.sleep(0.05)
            
        except Exception as e:
            print("Landing target processor error: {}".format(e))
            time.sleep(0.1)


def process_command(command):
    global centering_mode, landing_mode, latest_landing_target
    print("Processing command: {}".format(command))

    try:
        data = json.loads(command)
        action = data.get("action", "")
        value = data.get("value", "")

        if action == "takeoff":
            print("Action: Executing 'takeoff' command")
            target_altitude = float(data.get("altitude", takeoff_height))
            print("Target altitude set to {} meters".format(target_altitude))
            
            def takeoff_thread():
                try:
                    arm_and_takeoff(target_altitude)
                    print("Takeoff complete")
                    send_response(True, "Takeoff completed at {} meters".format(target_altitude), action)
                except Exception as e:
                    print("Takeoff failed: {}".format(e))
                    send_response(False, "Takeoff failed: {}".format(str(e)), action)
            
            t = Thread(target=takeoff_thread)
            t.daemon = True
            t.start()
            print("Takeoff initiated (running in background)")
            send_response(True, "Takeoff command received", action)

        elif action == "arm":
            print("Action: Executing 'arm' command")
            try:
                print("Setting flight mode to GUIDED...")
                set_mode("GUIDED")
                time.sleep(1)
                
                print("Arming motors...")
                arm_vehicle()
                time.sleep(2)
                
                print("Vehicle armed successfully")
                send_response(True, "Vehicle armed", action)
            except Exception as e:
                print("Arm error: {}".format(e))
                send_response(False, "Arm failed: {}".format(str(e)), action)

        elif action == "disarm":
            print("Action: Executing 'disarm' command")
            try:
                centering_mode = False
                landing_mode = False
                with landing_target_lock:
                    latest_landing_target = None
                
                with state_lock:
                    current_mode = vehicle_state["mode"]
                    current_alt = vehicle_state["altitude"]
                
                if current_mode != "LAND" and current_alt > 0.5:
                    print("Warning: Vehicle not landed. Switching to LAND mode first.")
                    set_mode("LAND")
                    send_response(True, "Landing before disarm", action)
                    return
                
                print("Disarming motors...")
                disarm_vehicle()
                time.sleep(1)
                
                print("Vehicle disarmed successfully")
                send_response(True, "Vehicle disarmed", action)
            except Exception as e:
                print("Disarm error: {}".format(e))
                send_response(False, "Disarm failed: {}".format(str(e)), action)

        elif action == "land":
            print("Action: Executing 'land' command")
            try:
                centering_mode = False
                landing_mode = False
                with landing_target_lock:
                    latest_landing_target = None
                set_mode("LAND")
                print("Regular landing initiated")
                send_response(True, "Landing initiated", action)
            except Exception as e:
                print("Land error: {}".format(e))
                send_response(False, "Land failed: {}".format(str(e)), action)

        elif action == "auto_land":
            print("Action: Executing 'auto_land' command")
            try:
                global landing_mode_initialized
                centering_mode = False
                landing_mode = True
                landing_mode_initialized = False
                
                set_mode("LAND")
                print("Switching to LAND mode - precision landing active")
                print("Auto-landing mode enabled - will guide to ArUco marker during descent")
                send_response(True, "Auto-landing enabled", action)
            except Exception as e:
                print("Auto-land error: {}".format(e))
                send_response(False, "Auto-land failed: {}".format(str(e)), action)

        elif action == "enable_centering":
            try:
                centering_mode = True
                print("Centering mode enabled")
                send_response(True, "Centering enabled", action)
            except Exception as e:
                print("Enable centering error: {}".format(e))
                send_response(False, "Enable centering failed: {}".format(str(e)), action)

        elif action == "disable_centering":
            try:
                centering_mode = False
                with landing_target_lock:
                    latest_landing_target = None
                print("Centering mode disabled and landing target data cleared")
                send_response(True, "Centering disabled", action)
            except Exception as e:
                print("Disable centering error: {}".format(e))
                send_response(False, "Disable centering failed: {}".format(str(e)), action)

        elif action == "move":
            print("Action: Executing 'move' command to: {}".format(value))
            try:
                coords = value.split(",")
                if len(coords) == 2:
                    lat = float(coords[0].strip())
                    lon = float(coords[1].strip())
                    
                    with state_lock:
                        current_alt = vehicle_state["altitude"]
                    
                    lat_int = int(lat * 1e7)
                    lon_int = int(lon * 1e7)
                    alt_int = int(current_alt * 1000)
                    
                    print("Moving to: lat={}, lon={}, alt={}".format(lat, lon, current_alt))
                    
                    mav.mav.set_position_target_global_int_send(
                        0,
                        mav.target_system,
                        mav.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        0b0000111111111000,
                        lat_int, lon_int, alt_int,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0
                    )
                    
                    send_response(True, "Navigation to ({}, {}) initiated".format(lat, lon), action)
                else:
                    print("Invalid coordinate format")
                    send_response(False, "Invalid format. Expected 'lat,lon'", action)
            except (ValueError, AttributeError) as e:
                print("Move error: {}".format(e))
                send_response(False, "Move failed: {}".format(str(e)), action)

        elif action == "emergency_stop":
            print("!!! EMERGENCY STOP ACTIVATED !!!")
            try:
                centering_mode = False
                landing_mode = False
                with landing_target_lock:
                    latest_landing_target = None
                set_mode("LAND")
                
                def emergency_disarm():
                    time.sleep(5)
                    with state_lock:
                        is_armed = vehicle_state["armed"]
                    if is_armed:
                        disarm_vehicle()
                        print("Emergency stop complete - vehicle disarmed")
                
                emergency_thread = Thread(target=emergency_disarm)
                emergency_thread.daemon = True
                emergency_thread.start()
                
                send_response(True, "Emergency stop activated", action)
            except Exception as e:
                print("Emergency stop error: {}".format(e))
                send_response(False, "Emergency stop failed: {}".format(str(e)), action)

        elif action == "manual_control":
            direction = data.get("direction", "")
            active = data.get("active", False)
            
            try:
                with state_lock:
                    is_armed = vehicle_state["armed"]
                    current_mode = vehicle_state["mode"]
                
                if not is_armed:
                    send_response(False, "Vehicle not armed", action)
                    return
                
                if current_mode != "GUIDED":
                    set_mode("GUIDED")
                    time.sleep(0.5)
                
                velocity_scale = 2.0
                yaw_rate = 30.0
                
                if active:
                    if direction == "forward":
                        send_local_ned_velocity(velocity_scale, 0, 0)
                        print("Manual control: Forward")
                    elif direction == "backward":
                        send_local_ned_velocity(-velocity_scale, 0, 0)
                        print("Manual control: Backward")
                    elif direction == "left":
                        send_local_ned_velocity(0, -velocity_scale, 0)
                        print("Manual control: Left")
                    elif direction == "right":
                        send_local_ned_velocity(0, velocity_scale, 0)
                        print("Manual control: Right")
                    elif direction == "up":
                        send_local_ned_velocity(0, 0, -velocity_scale)
                        print("Manual control: Up")
                    elif direction == "down":
                        send_local_ned_velocity(0, 0, velocity_scale)
                        print("Manual control: Down")
                    elif direction == "yaw-left":
                        mav.mav.command_long_send(
                            mav.target_system,
                            mav.target_component,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0,
                            0, -yaw_rate, 0, 1, 0, 0, 0
                        )
                        print("Manual control: Yaw Left")
                    elif direction == "yaw-right":
                        mav.mav.command_long_send(
                            mav.target_system,
                            mav.target_component,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0,
                            0, yaw_rate, 0, 1, 0, 0, 0
                        )
                        print("Manual control: Yaw Right")
                else:
                    send_local_ned_velocity(0, 0, 0)
                    print("Manual control: Stop {}".format(direction))
                    
            except Exception as e:
                print("Manual control error: {}".format(e))
                send_response(False, "Manual control failed: {}".format(str(e)), action)

        elif action == "status":
            with state_lock:
                print("\n" + "=" * 50)
                print("DRONE STATUS")
                print("=" * 50)
                print("  Armed: {}".format(vehicle_state["armed"]))
                print("  Mode: {}".format(vehicle_state["mode"]))
                print("  Altitude: {:.2f}m".format(vehicle_state["altitude"]))
                print("  Landing Mode: {}".format("ACTIVE" if landing_mode else "INACTIVE"))
                print("  Centering Mode: {}".format("ACTIVE" if centering_mode else "INACTIVE"))
                print("=" * 50 + "\n")

        else:
            print("Warning: Unknown command action: {}".format(action))
            send_response(False, "Unknown command: {}".format(action), action)

    except (json.JSONDecodeError, ValueError) as e:
        print("Warning: Received invalid message: {}".format(e))
        send_response(False, "Invalid message format", "unknown")


def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker with result code {}".format(rc))
    client.subscribe(command_topic)
    print("Subscribed to topic: {}".format(command_topic))
    client.subscribe(landing_target_topic)
    print("Subscribed to topic: {}".format(landing_target_topic))


def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected MQTT disconnect (code: {})".format(rc))
    else:
        print("MQTT client disconnected")


def on_message(client, userdata, msg):
    global latest_landing_target
    
    if msg.topic == command_topic:
        payload = msg.payload.decode()
        print("[{}] {}".format(msg.topic, payload))
        
        command_thread = Thread(target=process_command, args=(payload,))
        command_thread.daemon = True
        command_thread.start()
    
    elif msg.topic == landing_target_topic:
        try:
            target_data = json.loads(msg.payload.decode())
            with landing_target_lock:
                latest_landing_target = target_data
        except Exception as e:
            print("Failed to parse landing target: {}".format(e))


if __name__ == "__main__":
    try:
        print("=" * 60)
        print("LACC Drone - ROS Gazebo Flight Controller Client (PyMAVLink)")
        print("=" * 60)
        print("\nConfiguration:")
        print("  Vehicle: tcp:127.0.0.1:5763")
        print("  MQTT Broker: {}:{}".format(broker_address, broker_port))
        print("=" * 60)
        print()

        print("Setting precision landing parameters...")
        set_parameter(b"PLND_ENABLED", 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_parameter(b"PLND_TYPE", 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_parameter(b"PLND_EST_TYPE", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_parameter(b"LAND_SPEED", 20, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(1)

        listener_thread = Thread(target=message_listener)
        listener_thread.daemon = True
        listener_thread.start()
        print("MAVLink message listener started")
        time.sleep(2)

        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_disconnect = on_disconnect
        mqtt_client.on_message = on_message

        print("Connecting to MQTT broker at {}:{}...".format(broker_address, broker_port))
        mqtt_client.connect(broker_address, broker_port, 60)
        mqtt_client.loop_start()
        print("MQTT connected and ready for commands")
        print()
        
        monitor_thread = Thread(target=vehicle_state_monitor)
        monitor_thread.daemon = True
        monitor_thread.start()
        print("Vehicle state monitor started")
        
        telemetry_thread = Thread(target=telemetry_publisher)
        telemetry_thread.daemon = True
        telemetry_thread.start()
        print("Telemetry publisher started")
        
        landing_processor_thread = Thread(target=landing_target_processor)
        landing_processor_thread.daemon = True
        landing_processor_thread.start()
        print("Landing target processor started")
        print()
        
        print("Waiting for commands via MQTT...")
        print("  - Send MQTT messages to topic: {}".format(command_topic))
        print()

        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        print("Disconnected")

