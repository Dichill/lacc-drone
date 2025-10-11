# Drone Client Interface

A comprehensive client interface for debugging and testing a Pixhawk quadcopter using MAVLink/MAVProxy through the dronekit library.

## Features

- **Drone Connection**: Connect to Pixhawk via serial, TCP, or UDP
- **Real-time Telemetry**: Monitor drone status, location, battery, GPS, and more
- **Flight Control**: Arm, disarm, takeoff, land, and navigate
- **Server Integration**: Connect to Flask-SocketIO server for remote monitoring
- **Interactive CLI**: Debug and test with command-line interface
- **Type Safety**: Strict TypeScript-style type annotations for Python

## Installation

### 1. Install Dependencies

```bash
cd client
pip install -r requirements.txt
```

### 2. MAVProxy/MAVLink Setup

If using a physical Pixhawk:
- Connect via USB serial port (e.g., `/dev/ttyUSB0` on Linux, `/dev/tty.usbserial-*` on macOS)
- Ensure proper permissions: `sudo usermod -a -G dialout $USER`

For SITL (Software In The Loop) testing:
```bash
# Install SITL (requires ardupilot repo)
pip install MAVProxy
```

## Usage

### Basic Connection

**Serial Connection (Physical Drone):**
```bash
python client.py --connect /dev/ttyUSB0
```

**TCP Connection (SITL or Network):**
```bash
python client.py --connect tcp:127.0.0.1:5760
```

**UDP Connection:**
```bash
python client.py --connect udp:127.0.0.1:14550
```

### With Custom Server

```bash
python client.py --connect tcp:127.0.0.1:5760 --server http://192.168.1.100:5000
```

### Drone-Only Mode (No Server)

```bash
python client.py --connect /dev/ttyUSB0 --no-server
```

### Custom Baud Rate

```bash
python client.py --connect /dev/ttyUSB0 --baud 115200
```

## Interactive Commands

Once connected, you can use these commands:

| Command | Description |
|---------|-------------|
| `status` | Show current telemetry (location, attitude, battery, GPS) |
| `arm` | Arm the drone motors (required before flight) |
| `disarm` | Disarm the drone motors |
| `takeoff` | Take off to specified altitude (in meters) |
| `land` | Land at current location |
| `rtl` | Return to launch point |
| `goto` | Fly to specific GPS coordinates |
| `mode` | Change flight mode (GUIDED, STABILIZE, LAND, etc.) |
| `stream` | Toggle telemetry streaming to server |
| `help` | Show available commands |
| `quit` | Exit interactive mode |

## Example Session

```
$ python client.py --connect tcp:127.0.0.1:5760

[DRONE] Connecting to vehicle on: tcp:127.0.0.1:5760
[DRONE] Successfully connected to vehicle
[SERVER] Connected to server at http://localhost:5000

============================================================
DRONE CLIENT - INTERACTIVE MODE
============================================================

drone> status
============================================================
DRONE TELEMETRY
============================================================
Armed:          False
Mode:           STABILIZE
System Status:  STANDBY
Location:       Lat=-35.3632621, Lon=149.1652374
Altitude:       0.00m
...

drone> arm
[DRONE] Vehicle is armable
[DRONE] Setting mode to GUIDED
[DRONE] Arming motors...
[DRONE] ✓ Vehicle is now armed

drone> takeoff
Enter target altitude (meters): 10
[DRONE] Taking off to 10m...
[DRONE] Altitude: 3.2m / 10m
[DRONE] Altitude: 7.8m / 10m
[DRONE] ✓ Reached target altitude: 9.8m

drone> land
[DRONE] Landing...
[DRONE] ✓ Landed and disarmed

drone> quit
[CLIENT] Exiting interactive mode...
[CLIENT] Shutdown complete.
```

## Telemetry Data

The client provides comprehensive telemetry including:

- **Status**: Armed state, flight mode, system status, EKF health
- **Location**: GPS coordinates (lat/lon), altitude
- **Attitude**: Pitch, roll, yaw angles
- **Velocity**: 3D velocity vector (vx, vy, vz)
- **Navigation**: Heading, ground speed, air speed
- **Battery**: Voltage, current, remaining capacity
- **GPS**: Fix type, number of satellites

## Safety Notes

⚠️ **Important Safety Warnings:**

1. Always test in a safe, open area away from people and obstacles
2. Keep propellers away from people and objects when armed
3. Have a manual way to cut power in emergencies
4. Start with SITL simulation before flying real hardware
5. Ensure GPS lock before attempting outdoor flight
6. Monitor battery levels - land with at least 20% remaining
7. Familiarize yourself with RC transmitter failsafe settings

## Troubleshooting

### Connection Issues

**"Failed to connect to drone"**
- Verify correct port/connection string
- Check cable connections
- Ensure no other programs are using the port
- Try different baud rates (57600, 115200, 921600)

**"Timeout waiting for vehicle to become armable"**
- Check GPS lock (need 6+ satellites for outdoor mode)
- Verify pre-arm checks pass
- Ensure proper calibration (compass, accelerometer)
- Check battery voltage is sufficient

### Server Connection Issues

**"Failed to connect to server"**
- Verify server is running: `cd server && python server.py`
- Check firewall settings
- Ensure correct server URL
- Use `--no-server` flag if server not needed

## Integration with Server

The client integrates with the Flask-SocketIO server:

- **Telemetry Streaming**: Sends real-time drone data via `telemetry` event
- **Remote Commands**: Receives commands via `command` event
- **Video Feed**: Server can process video frames (see `server/routes/api.py`)

## Development

### Code Structure

```
client.py
├── DroneClient class
│   ├── Connection management (drone + server)
│   ├── Flight control methods
│   ├── Telemetry methods
│   └── Interactive CLI
└── main() - Entry point with argument parsing
```

### Adding Custom Commands

To add new commands, extend the `interactive_mode()` method and add corresponding handler methods in the `DroneClient` class.

## License

Part of LACCDRONE project - Pixhawk quadcopter control system.

## References

- [DroneKit Documentation](https://dronekit-python.readthedocs.io/)
- [MAVLink Protocol](https://mavlink.io/)
- [ArduPilot Documentation](https://ardupilot.org/)


