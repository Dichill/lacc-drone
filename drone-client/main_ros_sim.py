#!/usr/bin/env python

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
from threading import Thread, Lock

vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True)
vehicle.parameters["PLND_ENABLED"] = 1
vehicle.parameters["PLND_TYPE"] = 1
vehicle.parameters["PLND_EST_TYPE"] = 0
vehicle.parameters["LAND_SPEED"] = 20

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


def vehicle_state_monitor():
    global landing_mode, centering_mode, landing_mode_initialized
    print("Vehicle state monitor started")
    
    was_armed = False
    
    while True:
        try:
            is_armed = vehicle.armed
            
            if was_armed and not is_armed:
                if landing_mode or centering_mode:
                    print("Vehicle disarmed - auto-disabling landing/centering modes")
                    landing_mode = False
                    centering_mode = False
                    landing_mode_initialized = False
            
            was_armed = is_armed
            time.sleep(0.5)
            
        except Exception as e:
            print("State monitor error: {}".format(e))
            time.sleep(1)


def arm_and_takeoff(targetHeight):
    global landing_mode, centering_mode, landing_mode_initialized
    
    if landing_mode or centering_mode:
        print("Resetting landing/centering modes from previous flight")
        landing_mode = False
        centering_mode = False
        landing_mode_initialized = False
    
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != "GUIDED":
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle now in GUIDED mode. Have Fun!")

    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Look out! Virtual props are spinning!")

    vehicle.simple_takeoff(targetHeight)

    start_time = time.time()
    timeout = 30
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print("Current Altitude: %d" % current_alt)
        
        if current_alt >= 0.95 * targetHeight:
            print("Target altitude reached!")
            break
        
        if time.time() - start_time > timeout:
            print("Altitude check timed out after {} seconds".format(timeout))
            print("Current altitude: {} (target: {})".format(current_alt, targetHeight))
            print("Continuing anyway - altitude sensor may not be working in sim")
            break
        
        time.sleep(1)
    
    time.sleep(2)
    print("Takeoff complete - staying in GUIDED mode for manual control")
    print("Note: Switch to LOITER manually if you want GPS position hold")

    return None


def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, x, y, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


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
                telemetry = {
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
                        
                        if vehicle.mode != "LAND":
                            vehicle.mode = VehicleMode("LAND")
                            print("Switching to LAND mode...")
                        
                        current_alt = vehicle.location.global_relative_frame.alt
                        
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
                        if vehicle.mode.name != "GUIDED":
                            print("Centering mode requires GUIDED mode - switching from {}".format(vehicle.mode.name))
                            vehicle.mode = VehicleMode("GUIDED")
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
                                print("Centering: {} (vx={:.2f}, vy={:.2f}, marker@{:.0f},{:.0f})".format(
                                    " + ".join(direction_msg), vx, vy, center_x, center_y))
                            
                            send_local_ned_velocity(vx, vy, 0)
                            last_command_time = current_time
            
            time.sleep(0.05)
            
        except Exception as e:
            print("Landing target processor error: {}".format(e))
            time.sleep(0.1)


def process_command(command):
    global centering_mode, landing_mode
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
                if not vehicle.is_armable:
                    send_response(False, "Vehicle not armable", action)
                    return
                
                vehicle.mode = VehicleMode("GUIDED")
                while vehicle.mode != "GUIDED":
                    print("Waiting for GUIDED mode...")
                    time.sleep(0.5)
                
                vehicle.armed = True
                timeout = 10
                start = time.time()
                while not vehicle.armed and (time.time() - start) < timeout:
                    print("Waiting for vehicle to arm...")
                    time.sleep(0.5)
                
                if vehicle.armed:
                    print("Vehicle armed successfully")
                    send_response(True, "Vehicle armed", action)
                else:
                    print("Failed to arm vehicle")
                    send_response(False, "Arm timeout", action)
            except Exception as e:
                print("Arm error: {}".format(e))
                send_response(False, "Arm failed: {}".format(str(e)), action)

        elif action == "disarm":
            print("Action: Executing 'disarm' command")
            try:
                centering_mode = False
                landing_mode = False
                
                current_mode = vehicle.mode.name
                if current_mode != "LAND" and vehicle.location.global_relative_frame.alt > 0.5:
                    print("Warning: Vehicle not landed. Switching to LAND mode first.")
                    vehicle.mode = VehicleMode("LAND")
                    send_response(True, "Landing before disarm", action)
                    return
                
                vehicle.armed = False
                timeout = 10
                start = time.time()
                while vehicle.armed and (time.time() - start) < timeout:
                    print("Waiting for vehicle to disarm...")
                    time.sleep(0.5)
                
                if not vehicle.armed:
                    print("Vehicle disarmed successfully")
                    send_response(True, "Vehicle disarmed", action)
                else:
                    print("Failed to disarm vehicle")
                    send_response(False, "Disarm timeout - vehicle may not be landed", action)
            except Exception as e:
                print("Disarm error: {}".format(e))
                send_response(False, "Disarm failed: {}".format(str(e)), action)

        elif action == "land":
            print("Action: Executing 'land' command")
            try:
                centering_mode = False
                landing_mode = False
                vehicle.mode = VehicleMode("LAND")
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
                
                vehicle.mode = VehicleMode("LAND")
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
                print("Centering mode disabled")
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
                    
                    current_alt = vehicle.location.global_relative_frame.alt
                    target_location = LocationGlobalRelative(lat, lon, current_alt)
                    
                    print("Moving to: lat={}, lon={}, alt={}".format(lat, lon, current_alt))
                    vehicle.simple_goto(target_location)
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
                vehicle.mode = VehicleMode("LAND")
                
                def emergency_disarm():
                    time.sleep(5)
                    if vehicle.armed:
                        vehicle.armed = False
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
                if not vehicle.armed:
                    send_response(False, "Vehicle not armed", action)
                    return
                
                if vehicle.mode.name != "GUIDED":
                    vehicle.mode = VehicleMode("GUIDED")
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
                        msg = vehicle.message_factory.command_long_encode(
                            0, 0,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0,
                            0, -yaw_rate, 0, 1, 0, 0, 0
                        )
                        vehicle.send_mavlink(msg)
                        print("Manual control: Yaw Left")
                    elif direction == "yaw-right":
                        msg = vehicle.message_factory.command_long_encode(
                            0, 0,
                            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0,
                            0, yaw_rate, 0, 1, 0, 0, 0
                        )
                        vehicle.send_mavlink(msg)
                        print("Manual control: Yaw Right")
                else:
                    send_local_ned_velocity(0, 0, 0)
                    print("Manual control: Stop {}".format(direction))
                    
            except Exception as e:
                print("Manual control error: {}".format(e))
                send_response(False, "Manual control failed: {}".format(str(e)), action)

        elif action == "status":
            print("\n" + "=" * 50)
            print("DRONE STATUS")
            print("=" * 50)
            print("  Armed: {}".format(vehicle.armed))
            print("  Mode: {}".format(vehicle.mode.name))
            print("  Altitude: {:.2f}m".format(vehicle.location.global_relative_frame.alt))
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
        print("LACC Drone - ROS Gazebo Flight Controller Client")
        print("=" * 60)
        print("\nConfiguration:")
        print("  Vehicle: tcp:127.0.0.1:5763")
        print("  MQTT Broker: {}:{}".format(broker_address, broker_port))
        print("=" * 60)
        print()

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
