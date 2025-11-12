# LACC Drone Ground Station

A modern, NASA-inspired ground station dashboard for Los Angeles City College's drone operations. Built with Next.js, React, TypeScript, and MQTT communication protocol.

## Features

### 📡 MQTT Communication
- **Real-time Connection**: WebSocket-based MQTT connection to drone broker
- **Command Publishing**: Send commands to drone via `drone/commands` topic
- **Video Streaming**: Receive live camera feed via `camera/stream` topic  
- **ArUco Detection**: Real-time marker detection data via `drone/aruco_detection` topic
- **Automatic Reconnection**: Built-in reconnection logic with status monitoring

### 📹 Video & Detection
- **Live Camera Feed**: Real-time video stream from drone camera (base64 encoded)
- **ArUco Marker Detection**: Visual indicator for detected markers with:
  - Marker ID display
  - Distance measurement (meters)
  - Angle/orientation visualization
  - Auto-land capability when marker is detected

### 🎯 Telemetry Display
- **Altitude Gauge**: Circular gauge showing current altitude (0-150m) with warning thresholds
- **Ground Speed Gauge**: Displays current ground speed (0-25 m/s)
- **Distance to Waypoint**: Shows distance to next waypoint in mission
- **Heading/Yaw Gauge**: Compass-style display showing aircraft orientation (0-360°)
- **Vertical Speed Indicator**: Linear gauge showing climb/descent rate (-5 to +5 m/s)

### 🎮 Mission Control Panel
- **System Status**: Arm/Disarm controls for drone operations
- **Takeoff Control**: Configurable altitude takeoff (1-50m)
- **Navigation**: Move to destination waypoint
- **Landing Controls**:
  - Manual Landing: Standard descent procedure
  - Auto Landing (CV): Computer vision guided landing on ArUco marker
- **Marker Centering**: Enable/disable automatic marker centering
- **Emergency Stop**: Immediate system shutdown capability

### 🕹️ Manual Flight Controls
- **Directional Control**: D-pad style buttons for forward, backward, left, right movement
- **Throttle Control**: Vertical controls for altitude adjustment with real-time indicator
- **Yaw Control**: Rotate left/right for heading adjustment
- **Visual Feedback**: Active controls are highlighted with color-coded glow effects
- **Touch Support**: Works with both mouse and touch inputs for mobile/tablet use
- **Live Indicators**: Shows active control inputs in real-time

### 🎨 Design Features
- NASA-inspired dark theme with mission control aesthetics
- Real-time telemetry updates via MQTT
- Color-coded warning/danger thresholds
- Monospace typography for precise data display
- Responsive grid layout for desktop and mobile
- Status indicators for connection and system state

## Tech Stack

- **Framework**: Next.js 15.5.4 with App Router
- **UI Library**: React 19
- **Styling**: Tailwind CSS v4
- **Components**: Shadcn UI (New York style)
- **Icons**: Lucide React
- **Communication**: MQTT.js 5.14.1
- **Language**: TypeScript (strict mode)

## Getting Started

### Prerequisites
- Node.js 20+
- pnpm (package manager)
- MQTT Broker (e.g., Mosquitto)

### MQTT Broker Setup

#### Option 1: Local Simulation (Mosquitto)

Install Mosquitto MQTT broker:

```bash
# macOS
brew install mosquitto

# Start Mosquitto with WebSocket support
mosquitto -c /opt/homebrew/etc/mosquitto/mosquitto.conf
```

Configure Mosquitto for WebSocket support (`/opt/homebrew/etc/mosquitto/mosquitto.conf`):

```conf
listener 1883
protocol mqtt

listener 9001
protocol websockets

allow_anonymous true
```

#### Option 2: Real Drone Connection

Set `SIMULATION_MODE = false` in `lib/config.ts` and update the broker address to your drone's IP (default: `192.168.0.163`).

### Installation

```bash
# Install dependencies
pnpm install

# Run development server
pnpm dev
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

### Build for Production

```bash
# Create optimized production build
pnpm build

# Start production server
pnpm start
```

## Configuration

### Simulation vs Real Drone

Edit `lib/config.ts` to switch between simulation and real drone:

```typescript
// For simulation (localhost)
export const SIMULATION_MODE: boolean = true;

// For real drone
export const SIMULATION_MODE: boolean = false;
```

### MQTT Topics

The application uses the following MQTT topics (configured in `lib/config.ts`):

- **`drone/commands`**: Publish drone commands (takeoff, land, move, etc.)
- **`camera/stream`**: Subscribe to video stream (base64 encoded JPEG)
- **`drone/aruco_detection`**: Subscribe to ArUco marker detection data (JSON)

### Drone Commands

Commands are published as JSON to `drone/commands`:

```typescript
// Takeoff with altitude
{ "action": "takeoff", "altitude": 5.0 }

// Land
{ "action": "land" }

// Move to destination
{ "action": "move", "value": "waypoint_1" }

// Auto-land on ArUco marker
{ "action": "auto_land" }

// Enable marker centering
{ "action": "enable_centering" }

// Disable marker centering
{ "action": "disable_centering" }
```

## Project Structure

```
ground-station/
├── app/
│   ├── layout.tsx          # Root layout with dark theme
│   ├── page.tsx            # Main dashboard page (MQTT-enabled)
│   └── globals.css         # Global styles and NASA theme
├── components/
│   ├── gauges/
│   │   ├── circular-gauge.tsx      # Circular instrument display
│   │   ├── linear-gauge.tsx        # Vertical speed indicator
│   │   └── heading-gauge.tsx       # Compass/heading display
│   ├── control-panel.tsx           # Mission control interface (MQTT commands)
│   ├── manual-controls.tsx         # Manual flight controls
│   ├── mqtt-connection.tsx         # MQTT connection component
│   ├── video-stream.tsx            # Live video feed display
│   ├── aruco-detection.tsx         # ArUco marker detection display
│   └── ui/                         # Shadcn UI components
│       ├── button.tsx
│       ├── card.tsx
│       └── badge.tsx
├── hooks/
│   ├── use-mqtt.ts         # MQTT connection hook
│   ├── use-serial-port.ts  # Serial port hook (deprecated)
│   └── use-mavlink.ts      # MAVLink parser (deprecated)
├── lib/
│   ├── config.ts           # MQTT and system configuration
│   └── utils.ts            # Utility functions
└── types/
    └── serial.d.ts         # TypeScript type definitions
```

## MQTT Integration

### Python Sender Script

The repository includes `sender.py`, a Python script that demonstrates the MQTT communication protocol:

```bash
# Run interactive mode
python sender.py

# Send specific commands
python sender.py takeoff 5.0
python sender.py land
python sender.py move waypoint_1
```

The web application follows the same MQTT pattern as `sender.py` for consistency.

### Custom Hook: `useMQTT`

The `useMQTT` hook provides all MQTT functionality:

```typescript
const mqtt = useMQTT();

// Connection
await mqtt.connect();
await mqtt.disconnect();

// Send commands
await mqtt.sendTakeoff(5.0);
await mqtt.sendLand();
await mqtt.sendMove("waypoint_1");
await mqtt.sendAutoLand();
await mqtt.sendEnableCentering();
await mqtt.sendDisableCentering();

// Access data
const videoFrame = mqtt.videoFrame;      // Latest video frame
const detection = mqtt.arUcoDetection;   // ArUco detection data
const connected = mqtt.isConnected;      // Connection status
```

## Manual Control Integration

The manual controls provide callbacks for drone movement commands:

```typescript
const handleManualControl = (direction: string, active: boolean) => {
  // direction can be: "forward", "backward", "left", "right", 
  //                   "up", "down", "yaw-left", "yaw-right"
  // active: true when pressed, false when released
  
  if (active) {
    // Send command to drone to start moving in direction
    mqtt.sendCommand({ action: direction });
  } else {
    // Send command to drone to stop moving in direction
    mqtt.sendCommand({ action: "stop", value: direction });
  }
};
```

## Video Stream Format

Video frames are transmitted as base64-encoded JPEG images:

```typescript
// Sender (Python):
jpg_original = cv2.imencode('.jpg', frame)[1]
jpg_as_text = base64.b64encode(jpg_original).decode('utf-8')
client.publish("camera/stream", jpg_as_text)

// Receiver (Web App):
// Automatically decoded and displayed in VideoStream component
```

## ArUco Detection Format

ArUco detection data is transmitted as JSON:

```json
{
  "detected": true,
  "marker_id": 0,
  "distance": 2.5,
  "angle": 15.3
}
```

## Customization

### Gauge Thresholds
Modify warning and danger thresholds in `app/page.tsx`:

```typescript
<CircularGauge
  warningThreshold={120}  // Yellow warning
  dangerThreshold={140}   // Red danger
  dangerOnHigh={true}     // Danger when exceeding threshold
/>
```

### MQTT Configuration
Update broker settings in `lib/config.ts`:

```typescript
export const MQTT_CONFIG = {
  BROKER_ADDRESS: "192.168.1.100",  // Your broker address
  BROKER_PORT: 1883,                // MQTT port
  WS_PORT: 9001,                    // WebSocket port
};
```

### Color Scheme
Update colors in `app/globals.css` CSS variables for different themes.

## Safety Features

- Connection status monitoring with visual indicators
- Arm/Disarm safety system
- ArUco marker detection validation before auto-land
- Emergency stop capability
- Visual warning indicators
- Real-time telemetry validation
- Automatic MQTT reconnection

## Troubleshooting

### Cannot Connect to MQTT Broker

1. Verify Mosquitto is running: `brew services list`
2. Check WebSocket support in Mosquitto config
3. Ensure port 9001 is not blocked by firewall
4. Check console for connection errors

### No Video Stream

1. Verify sender.py is running and publishing to `camera/stream`
2. Check MQTT connection status in UI
3. Verify camera is active on drone
4. Check browser console for decoding errors

### ArUco Detection Not Working

1. Ensure ArUco markers are visible to camera
2. Verify detection data is publishing to `drone/aruco_detection`
3. Check marker size and lighting conditions
4. Verify OpenCV ArUco detection is configured in drone code

## Development

### Type Safety
All components use strict TypeScript with no `any` types, non-null assertions, or unknown casts.

### Code Style
- Double quotes for strings
- Template literals for string concatenation
- Comprehensive JSDoc comments
- Error checking and validation

### Testing MQTT Commands

Use the Python sender script to test commands:

```bash
# Test takeoff
python sender.py takeoff 5.0

# Test landing
python sender.py land

# Interactive mode for testing
python sender.py
```

## Contributing

This is a student project for Los Angeles City College. For questions or contributions, please contact the LACC Drone Team.

## License

Copyright © 2025 Los Angeles City College

---

**Ground Station v2.0** | MQTT-Enabled Mission Control Interface
