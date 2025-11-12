#!/usr/bin/env python

########IMPORTS#########
import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array

# MQTT imports
import paho.mqtt.client as mqtt
import json
import base64
from threading import Thread, Lock
try:
    import queue  # Python 3
except ImportError:
    import Queue as queue  # Python 2.7

#######VARIABLES########

vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True)
vehicle.parameters["PLND_ENABLED"] = 1
vehicle.parameters["PLND_TYPE"] = 1
vehicle.parameters["PLND_EST_TYPE"] = 0
vehicle.parameters["LAND_SPEED"] = 30  # cms/s

velocity = -0.5  # m/s
takeoff_height = 4  # m

########################

# MQTT Configuration
broker_address = "localhost"
broker_port = 1883
command_topic = "drone/commands"
stream_topic = "camera/stream"
aruco_topic = "drone/aruco_detection"
response_topic = "drone/responses"
telemetry_topic = "drone/telemetry"

# ROS Publisher
newimg_pub = rospy.Publisher("/camera/color/image_new", Image, queue_size=10)

# ArUco Configuration
id_to_find = 72  # arucoID
marker_size = 20  # CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)  # 62.2 for picam V2, 53.5 for V1
vertical_fov = 48.8 * (math.pi / 180)  # 48.8 for V2, 41.41 for V1

found_count = 0
notfound_count = 0

#############CAMERA INTRINSICS#######

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

#####
time_last = 0
time_to_wait = 0.05

# MQTT Global State
mqtt_client = None
centering_mode = False  # Track if drone should center on ArUco marker
landing_mode = False

image_queue = queue.Queue(maxsize=1)
processing_lock = Lock()

smoothed_x_ang = 0.0
smoothed_y_ang = 0.0
last_command_time = 0.0
command_rate_limit = 0.2

last_stream_time = 0.0
stream_rate_limit = 0.05
last_aruco_time = 0.0
aruco_rate_limit = 0.1
last_aruco_publish_time = 0.0
aruco_publish_rate_limit = 0.2
last_detection_data = {"detected": False, "markers": [], "timestamp": time.time()} 


################FUNCTIONS###############
def vehicle_state_monitor():
    global landing_mode, centering_mode
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
            
            was_armed = is_armed
            time.sleep(0.5)  
            
        except Exception as e:
            print("State monitor error: {}".format(e))
            time.sleep(1)


def arm_and_takeoff(targetHeight):
    global landing_mode, centering_mode
    
    if landing_mode or centering_mode:
        print("Resetting landing/centering modes from previous flight")
        landing_mode = False
        centering_mode = False
    
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
        
        # Check timeout
        if time.time() - start_time > timeout:
            print("Altitude check timed out after {} seconds".format(timeout))
            print("Current altitude: {} (target: {})".format(current_alt, targetHeight))
            print("Continuing anyway - altitude sensor may not be working in sim")
            break
        
        time.sleep(1)

    return None


# Send velocity command to drone
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


# MQTT Command Processing
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
                centering_mode = False
                landing_mode = True
                print("Auto-landing mode enabled")
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
    """MQTT connection callback."""
    print("Connected to MQTT Broker with result code {}".format(rc))
    client.subscribe(command_topic)
    print("Subscribed to topic: {}".format(command_topic))


def on_message(client, userdata, msg):
    """MQTT message callback."""
    payload = msg.payload.decode()
    print("[{}] {}".format(msg.topic, payload))
    process_command(payload)


def msg_receiver(message):
    global time_last, time_to_wait
    
    if time.time() - time_last > time_to_wait:
        try:
            while not image_queue.empty():
                try:
                    image_queue.get_nowait()
                except queue.Empty:
                    break
            
            image_queue.put_nowait(message)
            time_last = time.time()
        except queue.Full:
            pass


def process_images():
    global notfound_count, found_count, id_to_find
    global landing_mode, centering_mode, mqtt_client, last_command_time
    global last_stream_time, stream_rate_limit, last_aruco_time, aruco_rate_limit
    global last_aruco_publish_time, aruco_publish_rate_limit, last_detection_data
    
    print("Image processing thread started")
    
    while True:
        try:
            message = image_queue.get(timeout=1.0)
            
            np_data = rnp.numpify(message)
            current_time = time.time()
            
            should_process_aruco = (current_time - last_aruco_time) >= aruco_rate_limit
            
            was_detected = last_detection_data.get("detected", False)
            force_process = landing_mode or centering_mode
            
            adaptive_stream_rate = stream_rate_limit * 1.5 if was_detected else stream_rate_limit
            should_stream = (current_time - last_stream_time) >= adaptive_stream_rate
            should_publish_aruco = (current_time - last_aruco_publish_time) >= aruco_publish_rate_limit
            
            if should_process_aruco or force_process:
                gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
                (corners, ids, rejected) = aruco.detectMarkers(
                    image=gray_img, dictionary=aruco_dict, parameters=parameters
                )
                last_aruco_time = current_time
            else:
                ids = None
                corners = None

            detection_data = None
            
            try:
                if ids is not None:
                    if ids[0] == id_to_find:
                        ret = aruco.estimatePoseSingleMarkers(
                            corners,
                            marker_size,
                            cameraMatrix=np_camera_matrix,
                            distCoeffs=np_dist_coeff,
                        )
                        (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
                        x = "{:.2f}".format(tvec[0])
                        y = "{:.2f}".format(tvec[1])
                        z = "{:.2f}".format(tvec[2])

                        y_sum = 0
                        x_sum = 0

                        x_sum = (
                            corners[0][0][0][0]
                            + corners[0][0][1][0]
                            + corners[0][0][2][0]
                            + corners[0][0][3][0]
                        )
                        y_sum = (
                            corners[0][0][0][1]
                            + corners[0][0][1][1]
                            + corners[0][0][2][1]
                            + corners[0][0][3][1]
                        )

                        x_avg = x_sum / 4
                        y_avg = y_sum / 4

                        x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                        y_ang = (y_avg - vertical_res * 0.5) * vertical_fov / vertical_res

                        current_time = time.time()
                        
                        if landing_mode:
                            if vehicle.mode != "LAND":
                                vehicle.mode = VehicleMode("LAND")
                                print("Switching to LAND mode...")
                            
                            smoothed_x, smoothed_y = smooth_angular_position(x_ang, y_ang, alpha=0.2)
                            
                            if current_time - last_command_time >= command_rate_limit:
                                send_land_message(smoothed_x, smoothed_y)
                                last_command_time = current_time

                        elif centering_mode:
                            offset_x = (x_avg - horizontal_res / 2) / (horizontal_res / 2)
                            offset_y = (y_avg - vertical_res / 2) / (vertical_res / 2)

                            smoothed_x, smoothed_y = smooth_angular_position(offset_x, offset_y, alpha=0.4)

                            deadzone = 0.1
                            if abs(smoothed_x) < deadzone:
                                smoothed_x = 0.0
                            if abs(smoothed_y) < deadzone:
                                smoothed_y = 0.0

                            velocity_gain = 0.3  
                            vx = smoothed_y * velocity_gain
                            vy = smoothed_x * velocity_gain

                            if current_time - last_command_time >= command_rate_limit:
                                send_local_ned_velocity(vx, vy, 0)
                                last_command_time = current_time

                        marker_position = "MARKER POSITION: x=" + x + " y=" + y + " z=" + z

                        aruco.drawDetectedMarkers(np_data, corners)
                        aruco.drawAxis(
                            np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10
                        )
                        cv2.putText(
                            np_data, marker_position, (10, 50), 0, 0.7, (255, 0, 0), thickness=2
                        )
                        
                        if landing_mode:
                            cv2.putText(np_data, "LANDING MODE (SMOOTH)", (10, 30), 0, 0.6, (0, 0, 255), thickness=2)
                        elif centering_mode:
                            cv2.putText(np_data, "CENTERING MODE", (10, 30), 0, 0.6, (0, 255, 0), thickness=2)
                        print(marker_position)
                        print(
                            "FOUND COUNT: "
                            + str(found_count)
                            + " NOTFOUND COUNT: "
                            + str(notfound_count)
                        )

                        found_count = found_count + 1

                        marker_id = int(ids[0][0])
                        detection_data = {
                            "detected": True,
                            "marker_id": marker_id,
                            "markers": [
                                {
                                    "id": marker_id,
                                    "center_x": float(x_avg),
                                    "center_y": float(y_avg),
                                    "position": {
                                        "x": float(tvec[0]),
                                        "y": float(tvec[1]),
                                        "z": float(tvec[2]),
                                    },
                                    "corners": corners[0][0].tolist(),
                                }
                            ],
                            "timestamp": time.time(),
                        }
                    else:
                        notfound_count = notfound_count + 1
                        detection_data = {
                            "detected": False,
                            "markers": [],
                            "timestamp": time.time(),
                        }
                else:
                    notfound_count = notfound_count + 1
                    detection_data = {
                        "detected": False,
                        "markers": [],
                        "timestamp": time.time(),
                    }
            except Exception as e:
                print("Target likely not found")
                print(e)
                notfound_count = notfound_count + 1
                detection_data = {"detected": False, "markers": [], "timestamp": time.time()}

            if detection_data is not None:
                last_detection_data = detection_data
            
            if mqtt_client:
                try:
                    if should_publish_aruco or (detection_data is not None):
                        detection_json = json.dumps(last_detection_data)
                        mqtt_client.publish(aruco_topic, detection_json, qos=0)
                        last_aruco_publish_time = current_time
                    
                    if should_stream:
                        stream_img = cv2.resize(np_data, (640, 480), interpolation=cv2.INTER_AREA)
                        
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 35]
                        ret_encode, jpeg = cv2.imencode(".jpg", stream_img, encode_param)
                        if ret_encode:
                            jpg_as_text = base64.b64encode(jpeg).decode("utf-8")
                            mqtt_client.publish(stream_topic, jpg_as_text, qos=0)
                        last_stream_time = current_time
                except Exception as e:
                    print("MQTT publish error: {}".format(e))

            if not (landing_mode or centering_mode):
                try:
                    new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
                    newimg_pub.publish(new_msg)
                except Exception as e:
                    print("ROS publish error: {}".format(e))
                
        except queue.Empty:
            continue
        except Exception as e:
            print("Image processing error: {}".format(e))
            import traceback
            traceback.print_exc()


def subscriber():
    rospy.init_node("drone_node", anonymous=False)
    sub = rospy.Subscriber("/camera/color/image_raw", Image, msg_receiver)
    rospy.spin()


if __name__ == "__main__":
    try:
        print("=" * 60)
        print("LACC Drone - ROS Gazebo Simulation Client")
        print("=" * 60)
        print("\nConfiguration:")
        print("  Vehicle: tcp:127.0.0.1:5763")
        print("  MQTT Broker: {}:{}".format(broker_address, broker_port))
        print("  Target Marker ID: {}".format(id_to_find))
        print("  Marker Size: {} cm".format(marker_size))
        print("=" * 60)
        print()

        # Setup MQTT client
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
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
        
        processing_thread = Thread(target=process_images)
        processing_thread.daemon = True
        processing_thread.start()
        print("Image processing thread started")
        
        telemetry_thread = Thread(target=telemetry_publisher)
        telemetry_thread.daemon = True
        telemetry_thread.start()
        print("Telemetry publisher started")
        print()
        
        print("Waiting for commands via MQTT...")
        print("  - Use sender.py to send commands (takeoff, land, etc.)")
        print("  - Or send MQTT messages to topic: {}".format(command_topic))
        print()

        # Start ROS subscriber (this will block until Ctrl+C)
        subscriber()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        print("Disconnected")
