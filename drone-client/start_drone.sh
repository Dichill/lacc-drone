#!/bin/bash

echo "======================================"
echo "LACC Drone System Startup"
echo "======================================"
echo ""

if [ ! -d "env" ]; then
    echo "Error: Virtual environment not found"
    echo "Please create virtual environment first:"
    echo "  python3 -m venv env"
    echo "  source env/bin/activate"
    echo "  pip install -r requirements.txt"
    exit 1
fi

source env/bin/activate

if ! command -v mosquitto &> /dev/null; then
    echo "Warning: mosquitto not found. Make sure MQTT broker is running."
fi

mkdir -p records

echo "Starting Camera Streamer..."
python3 camera_streamer.py &
CAMERA_PID=$!
echo "Camera Streamer PID: $CAMERA_PID"

sleep 2

echo "Starting Flight Controller Client..."
python3 main.py &
MAIN_PID=$!
echo "Flight Controller Client PID: $MAIN_PID"

echo ""
echo "======================================"
echo "System Running"
echo "======================================"
echo "Camera Streamer: PID $CAMERA_PID"
echo "Flight Controller: PID $MAIN_PID"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "======================================"

trap "echo ''; echo 'Stopping all processes...'; kill $CAMERA_PID $MAIN_PID 2>/dev/null; exit" INT TERM

wait

