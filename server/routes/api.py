from flask import jsonify
from server import socketio
from . import api_bp
import numpy as np
import base64
import cv2
import os


@socketio.on("/feed")
def handle_video_frame(data):
    # Decode base64 image data
    image_data = base64.b64decode(data.split(",")[1])
    np_arr = np.frombuffer(image_data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Process the frame (e.g., apply filters, object detection)
    # For example, convert to grayscale:
    # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Encode processed frame back to base64 (if sending back)
    # _, buffer = cv2.imencode('.jpeg', gray_frame)
    # processed_image_data = base64.b64encode(buffer).decode('utf-8')
    # emit('processed_video_frame', 'data:image/jpeg;base64,' + processed_image_data)

    print("Received video frame from client")


@api_bp.route("/boop")
def boop():
    item = {"message": "beep"}
    return jsonify(item)
