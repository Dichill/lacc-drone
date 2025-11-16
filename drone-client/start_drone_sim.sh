#!/bin/bash

echo "======================================"
echo "LACC Drone ROS Simulation Startup"
echo "======================================"
echo ""

if ! command -v roscore &> /dev/null; then
    echo "Error: ROS not found. Make sure ROS is installed and sourced."
    exit 1
fi

if ! command -v mosquitto &> /dev/null; then
    echo "Warning: mosquitto not found. Make sure MQTT broker is running."
fi

echo "Starting ROS Camera Streamer..."
python camera_streamer_ros_sim.py &
CAMERA_PID=$!
echo "Camera Streamer PID: $CAMERA_PID"

sleep 2

echo "Starting Flight Controller Client..."
python main_ros_sim.py &
MAIN_PID=$!
echo "Flight Controller Client PID: $MAIN_PID"

echo ""
echo "======================================"
echo "Simulation System Running"
echo "======================================"
echo "Camera Streamer: PID $CAMERA_PID"
echo "Flight Controller: PID $MAIN_PID"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "======================================"

trap "echo ''; echo 'Stopping all processes...'; kill $CAMERA_PID $MAIN_PID 2>/dev/null; exit" INT TERM

wait

