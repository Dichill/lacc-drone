#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
import paho.mqtt.client as mqtt
import json
import base64
try:
    import queue
except ImportError:
    import Queue as queue

broker_address = "localhost"
broker_port = 1883
stream_topic = "camera/stream"
aruco_topic = "drone/aruco_detection"
landing_target_topic = "drone/landing_target"
camera_status_topic = "camera/status"

newimg_pub = rospy.Publisher("/camera/color/image_new", Image, queue_size=10)

id_to_find = 72
marker_size = 20

FRAME_WIDTH = 480
FRAME_HEIGHT = 360
STREAM_QUALITY = 28
STREAM_INTERVAL = 0.066
DETECTION_INTERVAL = 0.05
ARUCO_PUBLISH_INTERVAL = 0.1
TIME_BETWEEN_FRAMES = 0.02
APPLY_UNDISTORTION = False

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = FRAME_WIDTH
vertical_res = FRAME_HEIGHT
horizontal_fov = 62.2 * (math.pi / 180)
vertical_fov = 48.8 * (math.pi / 180)

found_count = 0
notfound_count = 0

dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

time_last = 0
time_to_wait = TIME_BETWEEN_FRAMES

mqtt_client = None
image_queue = queue.Queue(maxsize=2)

last_stream_time = 0.0
last_aruco_time = 0.0
last_aruco_publish_time = 0.0
last_detection_data = {"detected": False, "markers": [], "timestamp": time.time()}


def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker with result code {}".format(rc))
    print("Publishing to topics: {}, {}".format(stream_topic, aruco_topic))


def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected MQTT disconnect (code: {})".format(rc))
    else:
        print("MQTT client disconnected")


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
    global notfound_count, found_count, id_to_find, mqtt_client
    global last_stream_time, last_aruco_time, last_aruco_publish_time, last_detection_data
    
    print("Image processing thread started")
    
    while True:
        try:
            message = image_queue.get(timeout=1.0)
            
            np_data = rnp.numpify(message)
            current_time = time.time()
            resized_frame = cv2.resize(np_data, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_AREA)

            if APPLY_UNDISTORTION:
                resized_frame = cv2.undistort(resized_frame, np_camera_matrix, np_dist_coeff)
            
            should_process_aruco = (current_time - last_aruco_time) >= DETECTION_INTERVAL
            should_stream = (current_time - last_stream_time) >= STREAM_INTERVAL
            should_publish_aruco = (current_time - last_aruco_publish_time) >= ARUCO_PUBLISH_INTERVAL
            
            if should_process_aruco:
                gray_img = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
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

                        found_count = found_count + 1

                        marker_id = int(ids[0][0])
                        detection_data = {
                            "detected": True,
                            "marker_id": marker_id,
                            "x_angle": float(x_ang),
                            "y_angle": float(y_ang),
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
                        
                        try:
                            mqtt_client.publish(landing_target_topic, json.dumps({
                                "marker_id": marker_id,
                                "x_angle": float(x_ang),
                                "y_angle": float(y_ang),
                                "center_x": float(x_avg),
                                "center_y": float(y_avg),
                                "timestamp": time.time()
                            }), qos=0)
                        except Exception as e:
                            print("Landing target publish error: {}".format(e))
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
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY]
                        ret_encode, jpeg = cv2.imencode(".jpg", resized_frame, encode_param)
                        if ret_encode:
                            jpg_as_text = base64.b64encode(jpeg).decode("utf-8")
                            mqtt_client.publish(stream_topic, jpg_as_text, qos=0)
                        last_stream_time = current_time
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
    rospy.init_node("camera_streamer_node", anonymous=False)
    sub = rospy.Subscriber("/camera/color/image_raw", Image, msg_receiver)
    rospy.spin()


if __name__ == "__main__":
    from threading import Thread
    
    try:
        print("=" * 60)
        print("LACC Drone - ROS Camera Streamer")
        print("=" * 60)
        print("MQTT Broker: {}:{}".format(broker_address, broker_port))
        print("Target Marker ID: {}".format(id_to_find))
        print("=" * 60)
        print()

        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_disconnect = on_disconnect

        print("Connecting to MQTT broker at {}:{}...".format(broker_address, broker_port))
        mqtt_client.connect(broker_address, broker_port, 60)
        mqtt_client.loop_start()
        print("MQTT connected")
        print()
        
        mqtt_client.publish(camera_status_topic, json.dumps({"status": "running", "timestamp": time.time()}), qos=0)
        
        processing_thread = Thread(target=process_images)
        processing_thread.daemon = True
        processing_thread.start()
        print("Image processing thread started")
        print()
        
        print("Camera streamer running...")
        print()

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

