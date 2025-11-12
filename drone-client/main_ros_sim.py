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

    while True:
        print("Current Altitude: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= 0.95 * targetHeight:
            break
        time.sleep(1)
    print("Target altitude reached!")

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
            arm_and_takeoff(target_altitude)
            print("Takeoff complete")

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
    global notfound_count, found_count, time_last, time_to_wait, id_to_find
    global landing_mode, centering_mode, mqtt_client

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)  # Deserialize image data into array
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
                    x = "{:.2f}".format(tvec[0])  # Xerror/distance between camera and aruco in CM
                    y = "{:.2f}".format(tvec[1])  # Yerror/distance between camera and aruco in CM
                    z = "{:.2f}".format(tvec[2])  # Zerror/distance between camera and aruco in CM

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

                    # If in landing mode, send landing target
                    if landing_mode:
                        if vehicle.mode != "LAND":
                            vehicle.mode = VehicleMode("LAND")
                            while vehicle.mode != "LAND":
                                time.sleep(1)
                            print("Vehicle in LAND mode")
                            send_land_message(x_ang, y_ang)
                        else:
                            send_land_message(x_ang, y_ang)

                    # If in centering mode, adjust position
                    elif centering_mode:
                        offset_x = (x_avg - horizontal_res / 2) / (horizontal_res / 2)
                        offset_y = (y_avg - vertical_res / 2) / (vertical_res / 2)

                        deadzone = 0.1
                        if abs(offset_x) < deadzone:
                            offset_x = 0.0
                        if abs(offset_y) < deadzone:
                            offset_y = 0.0

                        velocity_gain = 0.5
                        vx = offset_y * velocity_gain
                        vy = offset_x * velocity_gain

                        send_local_ned_velocity(vx, vy, 0)

                    marker_position = "MARKER POSITION: x=" + x + " y=" + y + " z=" + z

                    aruco.drawDetectedMarkers(np_data, corners)
                    aruco.drawAxis(
                        np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10
                    )
                    # putText(image, text_to_draw,position,font_face,fontScale,color,thickness)
                    cv2.putText(
                        np_data, marker_position, (10, 50), 0, 0.7, (255, 0, 0), thickness=2
                    )
                    print(marker_position)
                    print(
                        "FOUND COUNT: "
                        + str(found_count)
                        + " NOTFOUND COUNT: "
                        + str(notfound_count)
                    )

                    found_count = found_count + 1

                    # Prepare detection data for MQTT
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

        # Publish detection data to MQTT
        if mqtt_client:
            detection_json = json.dumps(detection_data)
            mqtt_client.publish(aruco_topic, detection_json, qos=0)

            # Publish JPEG frame to MQTT stream
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            ret_encode, jpeg = cv2.imencode(".jpg", np_data, encode_param)
            if ret_encode:
                jpg_as_text = base64.b64encode(jpeg).decode("utf-8")
                mqtt_client.publish(stream_topic, jpg_as_text, qos=0)

        new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None


def subscriber():
    rospy.init_node("drone_node", anonymous=False)
    sub = rospy.Subscriber("/camera/color/image_raw", Image, msg_receiver)
    rospy.spin()


if __name__ == "__main__":
    try:
        # Setup MQTT client
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message

        print("Connecting to MQTT broker at {}:{}...".format(broker_address, broker_port))
        mqtt_client.connect(broker_address, broker_port, 60)
        mqtt_client.loop_start()
        print("MQTT connected")

        # Follow old_ros.py pattern: arm, takeoff, move, then subscribe
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        send_local_ned_velocity(0, velocity, 0)
        time.sleep(10)
        subscriber()
    except rospy.ROSInterruptException:
        pass
    finally:
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
