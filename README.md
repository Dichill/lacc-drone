# LACC Drone Project

**Team Members**: Dichill Tomarong, Fahat Yousuf, Paola Ramirez, Sebastian Park, Marco Colin

**Under the Guidance of** Professor Jayesh Bhakta

---

A precision landing drone system featuring ArUco marker detection and a modern web-based ground control station. This project supports both real hardware deployment and ROS Gazebo simulation for development and testing.

## System Architecture

### Real Hardware System

The system uses a Raspberry Pi companion computer connected to a Pixhawk flight controller for autonomous precision landing.

![Real Hardware System Architecture](public/images/Drone_Main.png)

### ROS Simulation System

For testing and development, the system can be run in a ROS Gazebo simulation environment.

![ROS Simulation Architecture](public/images/ROS%20Diagram.png)


### **Key Differences from ROS Sim:**

| Feature | ROS Simulation | Real Hardware |
|---------|----------------|---------------|
| **Camera** | Gazebo virtual camera → ROS topic | Raspberry Pi Camera → Picamera2 |
| **Connection** | TCP socket (127.0.0.1:5763) | Serial UART (/dev/serial0) |
| **Flight Controller** | SITL (Software in the Loop) | Physical Pixhawk/ArduPilot |
| **Communication** | DroneKit wrapper | Direct PyMAVLink |
| **Environment** | Virtual 3D world | Real world flight |

## Demo

Watch our web-based ground control station controlling the drone in ROS Gazebo simulation:

[![Web Interface Demo](https://img.youtube.com/vi/kPHsVJfyG78/0.jpg)](https://youtu.be/kPHsVJfyG78)

[View Demo Video](https://youtu.be/kPHsVJfyG78)

## Project Structure

```
LACCDRONE/
├── drone-client/          # Drone-side Python scripts
│   ├── main.py           # Flight controller (real hardware)
│   ├── main_ros_sim.py   # Flight controller (ROS simulation)
│   ├── camera_streamer.py           # Camera streamer (real hardware)
│   └── camera_streamer_ros_sim.py   # Camera streamer (ROS simulation)
│
└── ground-station/        # Next.js web application
    ├── app/              # Next.js pages
    ├── components/       # React components
    └── hooks/            # Custom React hooks
```

## Features

- **Precision Landing** - ArUco marker-based landing guidance
- **Manual Control** - Web-based manual flight controls
- **Real-time Telemetry** - Live flight data and status
- **Video Streaming** - Real-time camera feed with marker overlay
- **Ground Station** - Modern web interface for drone control

## Technologies

- **Flight Control**: PyMAVLink, ArduPilot
- **Vision**: OpenCV, ArUco markers, Picamera2
- **Communication**: MQTT, WebSockets
- **Ground Station**: Next.js, React, TypeScript
- **Simulation**: ROS, Gazebo, DroneKit

## Installation & Setup

### 🚁 Drone Client Setup (Raspberry Pi)

#### 1. Flash Raspberry Pi OS

Flash **Raspberry Pi OS (Legacy, 32-bit)** as it is compatible and much more stable with MAVLink and Serial communication with the Pixhawk 2.4.8.

#### 2. Enable SSH

```bash
sudo raspi-config
```

Navigate to "Interface Options" → "SSH" → Enable

#### 3. Install Camera Support

_Optional, only follow if you have a legacy camera_
Follow this guide to install IMX708 Arducam or other legacy cameras:

📹 [Camera Installation Guide](https://www.youtube.com/watch?v=l534zjr9Ys4)

#### 4. Install MAVProxy

Follow the official MAVLink installation guide:

📚 [MAVProxy Installation Guide](https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html#mavproxy-downloadinstalllinux)

#### 5. Setup Python Environment

Create a virtual environment with system site packages:

```bash
python3 -m venv --system-site-packages env
```

Activate the environment:

```bash
source env/bin/activate
```

Install required Python packages:

```bash
# picamera 2 is optional, only install if you have a legacy camera.
pip install pymavlink paho-mqtt opencv-contrib-python picamera2
```

#### 6. Install MQTT Broker

Install Mosquitto:

```bash
sudo apt install mosquitto mosquitto-clients
```

Configure Mosquitto:

```bash
sudo vim /etc/mosquitto/mosquitto.conf
```

Add these lines to the config:

```conf
# Standard MQTT on port 1883
listener 1883
protocol mqtt

# WebSocket support on port 9001 (required for browser)
listener 9001
protocol websockets

# Allow anonymous connections (for development)
allow_anonymous true
```

Restart Mosquitto:

```bash
sudo systemctl restart mosquitto
```

#### 7. Enable Serial Communication

Enable serial interface:

```bash
sudo raspi-config
```

Navigate to "Interface Options" → "Serial Port"
- Disable serial console
- Enable serial port hardware

Follow the Raspberry Pi MAVLink setup guide:

📚 [Raspberry Pi via MAVLink](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)

#### 8. Run the Drone Client

Make the startup script executable:

```bash
chmod +x start_drone.sh
```

Run the drone client:

```bash
./start_drone.sh
```

---

### 🌐 Ground Station Setup

#### 1. Install Node.js

Head over to https://nodejs.org/en and install Node.js.

#### 2. Install PNPM

Once done, in your terminal, install pnpm.

```bash
npm i -g pnpm
```

#### 3. Install Packages

in the ground-station directory, we need to install the packages required to run the web interface.

```bash
pnpm i
```

#### 4. All Set!

You can now run our web interface!

```bash
pnpm run dev
```

---

### 🧪 ROS Simulation Setup

The ROS Simulation Setup requires a certain setup that could only be found in Drone Dojo's Course. Unfortunately we can't really provide the necessary files as it is licensed under **Drone Dojo**, we highly recommend checking him out and his courses (_see credits below_) if you would still like to proceed. 

The installation of the **drone-client** is relatively similar though it does not require you to make a **Virtual Environment**. Instead, it is managed by packages from **catkin_make** e.g. **example_pkg** where you can put these files in the script folder:

```bash
main_ros_sim.py
camera_stream_ros_sim.py
```

And then you can run these commands to execute it to ROS. (_ArduCopter & Gazebo ROS should be running already!_)

```bash
rosrun example_pkg ./main_ros_sim.py
rosrun example_pkg ./camera_stream_ros_sim.py
```

**NOTE: MAKE SURE TO CHANGE THE IP ADDRESS IN THE LIB/CONFIG.TS OF THE GROUND STATION TO YOUR CONNECTED VIRTUAL MACHINE.**

## Official Documentation

If you want to see our whole documentation for the drone itself, you can head over to our [Docmost](https://doc.drakos.cc/share/jm4apukrza/p/lacc-drone-project-rbcaXCtKhc), though incomplete, we hope that it is somewhat of use to your next drone project.

## Credits
This project is the result of many of the open source projects that were somewhat combined together, without these projects, this would have been really hard to do.

- Caleb - also known as Drone Dojo provides amazing resources on how to make drones especially making precision landing drones, we highly recommend to check out his amazing work! https://dojofordrones.com/
- Carson Stark - https://github.com/Carson-Stark/AutonomousDrone
- Egnchen - https://github.com/egnchen/rasp-cv-tag-detection
- Rosetta Drone - https://github.com/RosettaDrone/vision-landing-2