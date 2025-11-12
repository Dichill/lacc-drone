#!/usr/bin/env python

########IMPORTS#########

from __future__ import print_function
from __future__ import division

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
time_to_wait = 0.1  # 100 ms

# MQTT Global State
mqtt_client = None
centering_mode = False  # Track if drone should center on ArUco marker
landing_mode = False  # Track if drone is in landing mode

# Image processing queue to avoid blocking ROS callback
image_queue = queue.Queue(maxsize=2) 
processing_lock = Lock()

smoothed_x_ang = 0.0
smoothed_y_ang = 0.0
last_command_time = 0.0
command_rate_limit = 0.2 


################FUNCTIONS###############
def arm_and_takeoff(targetHeight):
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


# MQTT Command Processing
def process_command(command):
    """Process incoming MQTT command."""
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
            
            # Run takeoff in a separate thread to avoid blocking MQTT
            def takeoff_thread():
                arm_and_takeoff(target_altitude)
                print("Takeoff complete")
            
            t = Thread(target=takeoff_thread)
            t.daemon = True
            t.start()
            print("Takeoff initiated (running in background)")

        elif action == "land":
            print("Action: Executing 'land' command")
            centering_mode = False
            landing_mode = False
            vehicle.mode = VehicleMode("LAND")
            time.sleep(1)

        elif action == "auto_land":
            print("Action: Executing 'auto_land' command")
            centering_mode = False
            landing_mode = True
            print("Landing mode enabled - will auto-land on marker")

        elif action == "enable_centering":
            centering_mode = True
            print("Centering mode enabled")

        elif action == "disable_centering":
            centering_mode = False
            print("Centering mode disabled")

        elif action == "move":
            print("Moved to {}".format(value))
        else:
            print("Warning: Unknown command action: {}".format(action))

    except (json.JSONDecodeError, ValueError) as e:
        print("Warning: Received invalid message: {}".format(e))


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
            image_queue.put_nowait(message)
            time_last = time.time()
        except queue.Full:
            pass


def process_images():
    global notfound_count, found_count, id_to_find
    global landing_mode, centering_mode, mqtt_client, last_command_time
    
    print("Image processing thread started")
    
    while True:
        try:
            message = image_queue.get(timeout=1.0)
            
            np_data = rnp.numpify(message) 
            gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

            ids = ""
            (corners, ids, rejected) = aruco.detectMarkers(
                image=gray_img, dictionary=aruco_dict, parameters=parameters
            )

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

                        detection_data = {
                            "detected": True,
                            "markers": [
                                {
                                    "id": int(ids[0][0]),
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

            if mqtt_client:
                try:
                    detection_json = json.dumps(detection_data)
                    mqtt_client.publish(aruco_topic, detection_json, qos=0)

                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
                    ret_encode, jpeg = cv2.imencode(".jpg", np_data, encode_param)
                    if ret_encode:
                        jpg_as_text = base64.b64encode(jpeg).decode("utf-8")
                        mqtt_client.publish(stream_topic, jpg_as_text, qos=0)
                except Exception as e:
                    print("MQTT publish error: {}".format(e))

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
        
        processing_thread = Thread(target=process_images)
        processing_thread.daemon = True
        processing_thread.start()
        print("Image processing thread started")
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
