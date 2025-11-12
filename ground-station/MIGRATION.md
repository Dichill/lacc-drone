# Migration Guide: Serial/MAVLink → MQTT

This document explains the changes made to migrate the Ground Station from Serial/MAVLink communication to MQTT protocol, following the pattern established in `sender.py`.

## Overview of Changes

### Before (Serial/MAVLink)
- Direct serial connection via Web Serial API
- MAVLink protocol parsing for telemetry
- Limited to devices with serial port access
- No video streaming capability
- No ArUco marker detection

### After (MQTT)
- WebSocket-based MQTT connection
- Publish/Subscribe messaging pattern
- Works from any device with network access
- Live camera feed streaming
- Real-time ArUco marker detection
- Command publishing to drone

## Architecture Changes

### Old Architecture
```
[Drone Serial Port] ←→ [Web Serial API] ←→ [MAVLink Parser] ←→ [React Components]
```

### New Architecture
```
[Drone] ←→ [MQTT Broker] ←→ [WebSocket] ←→ [React Components]
                ↓
         [camera/stream]
         [drone/aruco_detection]
         [drone/commands]
```

## File Changes

### New Files

#### Hooks
- **`hooks/use-mqtt.ts`**: MQTT connection hook (replaces `use-serial-port.ts` and `use-mavlink.ts`)
  - Manages MQTT client connection
  - Handles video stream subscription
  - Handles ArUco detection subscription
  - Provides command publishing methods

#### Components
- **`components/mqtt-connection.tsx`**: MQTT connection UI (replaces `serial-connection.tsx`)
  - Connect/disconnect to MQTT broker
  - Display connection status
  - Show ArUco detection summary
  - Display message count

- **`components/video-stream.tsx`**: Live camera feed display
  - Shows base64 encoded video frames
  - FPS counter
  - Automatic canvas rendering
  - Connection status indicators

- **`components/aruco-detection.tsx`**: Marker detection display
  - Shows detection status
  - Displays marker ID, distance, angle
  - Visual indicators for alignment
  - Auto-land availability status

#### Configuration
- **`lib/config.ts`**: Centralized MQTT configuration
  - Simulation mode toggle
  - Broker address and ports
  - MQTT topic definitions
  - Flight parameter defaults

- **`types/mqtt.d.ts`**: TypeScript type definitions for MQTT
  - Command types
  - Detection data structures
  - Video frame formats
  - Telemetry interfaces

#### Documentation
- **`SETUP.md`**: Quick setup guide for MQTT broker
- **`MIGRATION.md`**: This file

### Modified Files

#### `components/control-panel.tsx`
**Before:**
- Simple callback for landing modes
- No direct drone communication

**After:**
- Receives `mqtt` connection object as prop
- Sends actual MQTT commands:
  - Takeoff with configurable altitude
  - Manual land
  - Auto-land (when ArUco detected)
  - Move to destination
  - Enable/disable marker centering
- Real-time ArUco detection integration
- Input validation and state management

#### `app/page.tsx`
**Before:**
```typescript
const { connect, disconnect, onData } = useSerialPort(57600);
const { processData } = useMAVLink();
```

**After:**
```typescript
const mqtt = useMQTT();
// Access: mqtt.connect(), mqtt.videoFrame, mqtt.arUcoDetection, etc.
```

**Changes:**
- Replaced Serial/MAVLink hooks with `useMQTT`
- Added video stream display
- Added ArUco detection display
- Updated connection status to use MQTT
- Reorganized layout for video-first experience

#### `package.json`
**Added:**
```json
"mqtt": "5.14.1"
```

#### `README.md`
- Completely rewritten to focus on MQTT
- Added MQTT broker setup instructions
- Documented all MQTT topics and commands
- Added troubleshooting section
- Updated feature list

### Deprecated Files

The following files are no longer used but kept for reference:

- `hooks/use-serial-port.ts` - Replaced by `use-mqtt.ts`
- `hooks/use-mavlink.ts` - MAVLink parsing no longer needed
- `components/serial-connection.tsx` - Replaced by `mqtt-connection.tsx`
- `types/serial.d.ts` - Serial API types no longer needed

**Note:** You can safely delete these files if you don't need them for reference.

## API Changes

### Connection Management

**Before:**
```typescript
const { connect, disconnect, isConnected } = useSerialPort(57600);

await connect();    // Opens serial port dialog
await disconnect(); // Closes serial port
```

**After:**
```typescript
const { connect, disconnect, isConnected } = useMQTT();

await connect();    // Connects to MQTT broker (no dialog)
await disconnect(); // Disconnects from broker
```

### Sending Commands

**Before:**
```typescript
// No direct command sending - would need to implement MAVLink command encoding
```

**After:**
```typescript
const mqtt = useMQTT();

// Simple, high-level command methods
await mqtt.sendTakeoff(5.0);
await mqtt.sendLand();
await mqtt.sendMove("waypoint_1");
await mqtt.sendAutoLand();
await mqtt.sendEnableCentering();
await mqtt.sendDisableCentering();

// Or send custom commands
await mqtt.sendCommand({
  action: "custom_action",
  value: "custom_value"
});
```

### Receiving Data

**Before:**
```typescript
const { lastMessage } = useMAVLink();

// Process MAVLink message
if (lastMessage.header.msgid === 33) {
  // GLOBAL_POSITION_INT
  const altitude = lastMessage.payload.alt / 1000;
}
```

**After:**
```typescript
const mqtt = useMQTT();

// Video frame
const videoFrame = mqtt.videoFrame;
// { data: "base64...", timestamp: 1234567890 }

// ArUco detection
const detection = mqtt.arUcoDetection;
// { detected: true, marker_id: 0, distance: 2.5, angle: 15.3 }

// Connection state
const connected = mqtt.isConnected;
const messageCount = mqtt.messageCount;
```

## Component Integration Changes

### Before: Serial Connection Component
```typescript
<SerialConnection onTelemetryUpdate={handleTelemetryUpdate} />
```

### After: MQTT Connection Component
```typescript
<MQTTConnection />
```

### Before: Control Panel
```typescript
<ControlPanel onLanding={handleLanding} />

const handleLanding = (mode: string) => {
  console.log(`Landing mode: ${mode}`);
  // No actual drone communication
};
```

### After: Control Panel
```typescript
const mqtt = useMQTT();

<ControlPanel 
  mqtt={mqtt}
  arUcoDetection={mqtt.arUcoDetection}
/>

// Commands are sent automatically when buttons are clicked
// No additional handlers needed
```

## Configuration Changes

### Before
- Hardcoded serial baud rate (57600)
- No configuration file

### After
Centralized configuration in `lib/config.ts`:

```typescript
// Toggle simulation mode
export const SIMULATION_MODE: boolean = true;

// MQTT broker settings
export const MQTT_CONFIG = {
  BROKER_ADDRESS: SIMULATION_MODE ? "localhost" : "192.168.0.163",
  BROKER_PORT: 1883,
  WS_PORT: 9001,
};

// MQTT topics
export const MQTT_TOPICS = {
  COMMAND: "drone/commands",
  STREAM: "camera/stream",
  ARUCO: "drone/aruco_detection",
};
```

## MQTT Protocol Details

### Topics Used

| Topic | Type | Format | Description |
|-------|------|--------|-------------|
| `drone/commands` | Publish | JSON | Send commands to drone |
| `camera/stream` | Subscribe | Base64 | Receive video frames (JPEG) |
| `drone/aruco_detection` | Subscribe | JSON | Receive marker detection data |

### Message Formats

#### Commands (Published)
```json
// Takeoff
{"action": "takeoff", "altitude": 5.0}

// Land
{"action": "land"}

// Move
{"action": "move", "value": "waypoint_1"}

// Auto-land
{"action": "auto_land"}

// Enable centering
{"action": "enable_centering"}

// Disable centering
{"action": "disable_centering"}
```

#### Video Stream (Subscribed)
```
Base64-encoded JPEG image string
Example: "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBD..."
```

#### ArUco Detection (Subscribed)
```json
{
  "detected": true,
  "marker_id": 0,
  "distance": 2.5,
  "angle": 15.3
}
```

## Consistency with sender.py

The web application now follows the exact same MQTT pattern as `sender.py`:

### Python (sender.py)
```python
# Configuration
SIMULATION_MODE = True
BROKER_ADDRESS = "localhost" if SIMULATION_MODE else "192.168.0.163"
COMMAND_TOPIC = "drone/commands"
STREAM_TOPIC = "camera/stream"
ARUCO_TOPIC = "drone/aruco_detection"

# Send command
def send_takeoff(client, altitude=5.0):
    return send_command(client, "takeoff", altitude=altitude)
```

### TypeScript (use-mqtt.ts)
```typescript
// Configuration (lib/config.ts)
export const SIMULATION_MODE: boolean = true;
export const MQTT_CONFIG = {
  BROKER_ADDRESS: SIMULATION_MODE ? "localhost" : "192.168.0.163",
};
export const MQTT_TOPICS = {
  COMMAND: "drone/commands",
  STREAM: "camera/stream",
  ARUCO: "drone/aruco_detection",
};

// Send command (use-mqtt.ts)
const sendTakeoff = async (altitude: number = 5.0): Promise<boolean> => {
  return sendCommand({ action: "takeoff", altitude });
};
```

## Testing the Migration

### 1. Start MQTT Broker
```bash
brew services start mosquitto
# or
mosquitto -c /opt/homebrew/etc/mosquitto/mosquitto.conf
```

### 2. Start Ground Station
```bash
pnpm dev
```

### 3. Test with sender.py
```bash
python sender.py
```

### 4. Verify Functionality
- [ ] MQTT connection works
- [ ] Commands are sent and received
- [ ] Video stream displays (if sender.py provides it)
- [ ] ArUco detection updates (if markers are detected)
- [ ] All buttons and controls work
- [ ] Connection status updates correctly

## Rollback Instructions

If you need to rollback to Serial/MAVLink:

1. Restore old `app/page.tsx` from git:
   ```bash
   git checkout HEAD~1 -- app/page.tsx
   ```

2. Use old components:
   ```typescript
   import { SerialConnection } from "@/components/serial-connection";
   ```

3. Remove MQTT dependency:
   ```bash
   pnpm remove mqtt
   ```

However, note that the new MQTT architecture provides:
- ✅ Better compatibility (works on any device)
- ✅ Video streaming
- ✅ ArUco detection
- ✅ Easier command sending
- ✅ More reliable connection

## Next Steps

### Recommended Enhancements

1. **Add Telemetry via MQTT**
   - Create `drone/telemetry` topic
   - Publish altitude, speed, position data
   - Update gauges with real data

2. **Add Authentication**
   - Configure MQTT username/password
   - Update connection options in `use-mqtt.ts`

3. **Add TLS/SSL**
   - Use secure WebSocket (`wss://`)
   - Configure Mosquitto with certificates

4. **Add Recording**
   - Record video stream to file
   - Log commands and responses
   - Export telemetry data

5. **Add Mission Planning**
   - Define waypoints
   - Upload missions via MQTT
   - Track progress

## Support

For questions about the migration:
- Review `SETUP.md` for configuration help
- Check `README.md` for feature documentation
- Compare code with `sender.py` for MQTT patterns
- Contact LACC Drone Team

---

**Migration Completed**: January 2025  
**Updated By**: Ground Station Development Team  
**Version**: 2.0.0

