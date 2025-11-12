# Ground Station Setup Guide

Quick start guide for setting up the LACC Drone Ground Station with MQTT communication.

## Prerequisites

1. **Node.js 20+** and **pnpm**
   ```bash
   node --version  # Should be 20 or higher
   pnpm --version  # Package manager
   ```

2. **MQTT Broker** (Mosquitto recommended)
   ```bash
   # macOS
   brew install mosquitto
   
   # Ubuntu/Debian
   sudo apt-get install mosquitto mosquitto-clients
   ```

## Quick Setup

### Step 1: Install Dependencies

```bash
cd ground-station
pnpm install
```

### Step 2: Configure MQTT Broker

#### For Simulation Mode (Localhost)

1. Create or edit Mosquitto config file:
   ```bash
   # macOS
   nano /opt/homebrew/etc/mosquitto/mosquitto.conf
   
   # Linux
   sudo nano /etc/mosquitto/mosquitto.conf
   ```

2. Add WebSocket support:
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

3. Start Mosquitto:
   ```bash
   # macOS
   brew services start mosquitto
   # or
   mosquitto -c /opt/homebrew/etc/mosquitto/mosquitto.conf
   
   # Linux
   sudo systemctl start mosquitto
   ```

4. Verify Mosquitto is running:
   ```bash
   # macOS
   brew services list
   
   # Linux
   sudo systemctl status mosquitto
   ```

#### For Real Drone Mode

1. Edit `lib/config.ts`:
   ```typescript
   export const SIMULATION_MODE: boolean = false;  // Set to false
   ```

2. Update broker address if needed:
   ```typescript
   export const MQTT_CONFIG = {
     BROKER_ADDRESS: SIMULATION_MODE ? "localhost" : "192.168.0.163",  // Update IP
     // ...
   };
   ```

### Step 3: Start Ground Station

```bash
pnpm dev
```

Open [http://localhost:3000](http://localhost:3000)

### Step 4: Test Connection

1. Click "Connect MQTT" button in the UI
2. Status should change to "Connected"
3. You should see "Messages Received" counter

## Testing MQTT Communication

### Option 1: Python Sender Script

Use the included `sender.py` to test commands and video streaming:

```bash
# Interactive mode
python sender.py

# Direct commands
python sender.py takeoff 5.0
python sender.py land
```

### Option 2: Mosquitto Command Line

Test MQTT topics manually:

```bash
# Subscribe to commands (in one terminal)
mosquitto_sub -h localhost -p 1883 -t "drone/commands" -v

# Publish test video frame (in another terminal)
echo "test_base64_image_data" | mosquitto_pub -h localhost -p 1883 -t "camera/stream" -l

# Publish test ArUco detection
mosquitto_pub -h localhost -p 1883 -t "drone/aruco_detection" -m '{"detected": true, "marker_id": 0, "distance": 2.5, "angle": 15.3}'
```

## Common Issues

### Issue: Cannot connect to MQTT broker

**Solution:**
1. Check if Mosquitto is running: `brew services list` or `sudo systemctl status mosquitto`
2. Verify WebSocket listener on port 9001: `netstat -an | grep 9001`
3. Check firewall settings
4. Look at browser console for specific error messages

### Issue: Connected but no video stream

**Solution:**
1. Verify sender is publishing to `camera/stream` topic
2. Check if video data is base64 encoded
3. Look at browser console for decoding errors
4. Test with mosquitto_sub: `mosquitto_sub -h localhost -t "camera/stream"`

### Issue: ArUco detection not showing

**Solution:**
1. Verify data is being published to `drone/aruco_detection`
2. Check JSON format matches expected structure
3. Test with mosquitto_pub (see example above)
4. Check browser console for parsing errors

### Issue: Commands not reaching drone

**Solution:**
1. Verify drone is subscribed to `drone/commands` topic
2. Check MQTT QoS settings (should be 1)
3. Monitor commands with: `mosquitto_sub -h localhost -t "drone/commands" -v`
4. Verify JSON format of commands

## MQTT Topics Reference

| Topic | Direction | Format | Description |
|-------|-----------|--------|-------------|
| `drone/commands` | Publish | JSON | Send commands to drone |
| `camera/stream` | Subscribe | Base64 | Receive video frames |
| `drone/aruco_detection` | Subscribe | JSON | Receive marker detection data |

### Command Examples

```json
// Takeoff
{"action": "takeoff", "altitude": 5.0}

// Land
{"action": "land"}

// Move
{"action": "move", "value": "waypoint_1"}

// Auto-land (requires ArUco marker detected)
{"action": "auto_land"}

// Enable marker centering
{"action": "enable_centering"}

// Disable marker centering
{"action": "disable_centering"}
```

### Video Stream Format

```python
# Python sender example
import base64
import cv2

# Capture or read frame
frame = cv2.imread("test.jpg")
_, buffer = cv2.imencode('.jpg', frame)
jpg_as_text = base64.b64encode(buffer).decode('utf-8')

# Publish to MQTT
client.publish("camera/stream", jpg_as_text)
```

### ArUco Detection Format

```json
{
  "detected": true,
  "marker_id": 0,
  "distance": 2.5,
  "angle": 15.3
}
```

## Development Workflow

1. **Start Mosquitto** (if not already running)
   ```bash
   brew services start mosquitto
   ```

2. **Start Ground Station**
   ```bash
   pnpm dev
   ```

3. **Start Drone Simulator or Real Drone**
   ```bash
   python sender.py  # Or your drone control script
   ```

4. **Connect in UI**
   - Click "Connect MQTT" button
   - Verify connection status

5. **Test Commands**
   - Use Mission Control panel to send commands
   - Monitor with: `mosquitto_sub -h localhost -t "drone/commands" -v`

## Production Deployment

For production use:

1. **Build optimized version**
   ```bash
   pnpm build
   pnpm start
   ```

2. **Use secure MQTT (TLS)**
   - Configure Mosquitto with TLS certificates
   - Update broker URL to use `wss://` instead of `ws://`

3. **Disable anonymous access**
   - Set up MQTT authentication
   - Configure username/password in Mosquitto
   - Update client connection options

4. **Set up proper network**
   - Configure firewall rules
   - Set up VPN if needed for remote access
   - Use static IP for drone

## Next Steps

- Integrate real telemetry data (altitude, speed, etc.)
- Add recording functionality for video stream
- Implement mission planning features
- Add data logging and analytics
- Integrate additional sensors

## Support

For issues or questions:
- Check browser console for errors
- Review Mosquitto logs: `tail -f /opt/homebrew/var/log/mosquitto/mosquitto.log`
- Contact LACC Drone Team

---

**Last Updated**: January 2025

