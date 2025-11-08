import paho.mqtt.client as mqtt
import json
import time
import sys
import cv2
import numpy as np
import base64
from typing import Dict, Optional
from threading import Thread


BROKER_ADDRESS: str = "192.168.0.163"
BROKER_PORT: int = 1883
COMMAND_TOPIC: str = "drone/commands"
STREAM_TOPIC: str = "camera/stream"
ARUCO_TOPIC: str = "drone/aruco_detection"

latest_frame: Optional[np.ndarray] = None
aruco_detection_data: Optional[Dict] = None
video_window_active: bool = False


def on_connect(client: mqtt.Client, userdata: object, flags: Dict, rc: int) -> None:
    if rc == 0:
        print(f"✓ Connected to MQTT broker at {BROKER_ADDRESS}:{BROKER_PORT}")
        client.subscribe(STREAM_TOPIC)
        client.subscribe(ARUCO_TOPIC)
        print(f"✓ Subscribed to video stream: {STREAM_TOPIC}")
        print(f"✓ Subscribed to ArUco detection: {ARUCO_TOPIC}")
    else:
        print(f"✗ Connection failed with result code {rc}")


def on_publish(client: mqtt.Client, userdata: object, mid: int) -> None:
    print(f"✓ Message published (ID: {mid})")


def on_message(client: mqtt.Client, userdata: object, msg) -> None:
    """Handle incoming MQTT messages for video stream and ArUco detection."""
    global latest_frame, aruco_detection_data
    
    try:
        if msg.topic == STREAM_TOPIC:
            jpg_as_text: str = msg.payload.decode("utf-8")
            jpg_original: bytes = base64.b64decode(jpg_as_text)
            jpg_as_np: np.ndarray = np.frombuffer(jpg_original, dtype=np.uint8)
            latest_frame = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
            
        elif msg.topic == ARUCO_TOPIC:
            detection_json: str = msg.payload.decode("utf-8")
            aruco_detection_data = json.loads(detection_json)
            
    except Exception as e:
        print(f"✗ Error processing message from {msg.topic}: {e}")


def send_command(client: mqtt.Client, action: str, value: Optional[str] = None, **kwargs) -> bool:
    try:
        command: Dict = {"action": action}

        if value is not None:
            command["value"] = value
        
        command.update(kwargs)

        payload: str = json.dumps(command)

        print(f"\n→ Sending command: {payload}")
        result = client.publish(COMMAND_TOPIC, payload, qos=1)

        result.wait_for_publish()

        return result.is_published()

    except json.JSONDecodeError as e:
        print(f"✗ Error encoding command to JSON: {e}")
        return False
    except Exception as e:
        print(f"✗ Error sending command: {e}")
        return False


def send_takeoff(client: mqtt.Client, altitude: float = 5.0) -> bool:
    """Send takeoff command with optional altitude parameter (default: 5.0 meters)"""
    return send_command(client, "takeoff", altitude=altitude)


def send_land(client: mqtt.Client) -> bool:
    return send_command(client, "land")


def send_move(client: mqtt.Client, destination: str) -> bool:
    return send_command(client, "move", destination)


def send_auto_land(client: mqtt.Client) -> bool:
    """Send auto-land command to land on detected ArUco marker."""
    return send_command(client, "auto_land")


def send_enable_centering(client: mqtt.Client) -> bool:
    """Enable centering mode to track ArUco marker."""
    return send_command(client, "enable_centering")


def send_disable_centering(client: mqtt.Client) -> bool:
    """Disable centering mode."""
    return send_command(client, "disable_centering")


def display_video_stream() -> None:
    """Display video stream with ArUco marker overlays in a separate thread."""
    global latest_frame, aruco_detection_data, video_window_active
    
    video_window_active = True
    window_name: str = "Drone Camera Feed - Press 'q' to close"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)
    
    print("\n📹 Video feed window opened")
    print("Controls:")
    print("  'q' - Close video window")
    print("  'c' - Enable centering mode")
    print("  'd' - Disable centering mode")
    print("  'l' - Auto-land on marker\n")
    
    while video_window_active:
        if latest_frame is not None:
            display_frame: np.ndarray = latest_frame.copy()
            
            if aruco_detection_data is not None and aruco_detection_data.get("detected", False):
                markers = aruco_detection_data.get("markers", [])
                
                for marker in markers:
                    marker_id: int = marker.get("id", -1)
                    center_x: float = marker.get("center_x", 0)
                    center_y: float = marker.get("center_y", 0)
                    
                    center_point = (int(center_x), int(center_y))
                    cv2.circle(display_frame, center_point, 5, (0, 255, 0), -1)
                    
                    cv2.line(display_frame, (int(center_x) - 20, int(center_y)), 
                            (int(center_x) + 20, int(center_y)), (0, 255, 0), 2)
                    cv2.line(display_frame, (int(center_x), int(center_y) - 20), 
                            (int(center_x), int(center_y) + 20), (0, 255, 0), 2)
                    
                    text: str = f"ID: {marker_id}"
                    cv2.putText(display_frame, text, (int(center_x) + 10, int(center_y) - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                status_text: str = f"Markers Detected: {len(markers)}"
                cv2.putText(display_frame, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "No ArUco Markers Detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            height, width = display_frame.shape[:2]
            center_x: int = width // 2
            center_y: int = height // 2
            cv2.line(display_frame, (center_x - 30, center_y), 
                    (center_x + 30, center_y), (255, 0, 0), 1)
            cv2.line(display_frame, (center_x, center_y - 30), 
                    (center_x, center_y + 30), (255, 0, 0), 1)
            
            cv2.imshow(window_name, display_frame)
        
        key: int = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("✓ Closing video window...")
            break
        elif key == ord("c"):
            print("→ Enabling centering mode...")
        elif key == ord("d"):
            print("→ Disabling centering mode...")
        elif key == ord("l"):
            print("→ Initiating auto-land...")
    
    video_window_active = False
    cv2.destroyAllWindows()
    print("✓ Video feed closed")


def interactive_mode(client: mqtt.Client) -> None:
    global video_window_active
    
    print("\n" + "=" * 60)
    print("LACC DRONE | CLI")
    print("=" * 60)
    print("\nAvailable commands:")
    print("  1. takeoff [altitude]  - Command the drone to take off (default: 5.0m)")
    print("  2. land                - Command the drone to land")
    print("  3. move <dest>         - Command the drone to move to a destination")
    print("  4. auto_land           - Auto-land on detected ArUco marker")
    print("  5. enable_centering    - Enable marker centering mode")
    print("  6. disable_centering   - Disable marker centering mode")
    print("  7. video               - Open/close video feed window")
    print("  8. custom              - Send a custom JSON command")
    print("  9. quit                - Exit the program")
    print("\n" + "=" * 60 + "\n")
    
    video_thread: Thread = Thread(target=display_video_stream, daemon=True)
    video_thread.start()

    while True:
        try:
            command_input: str = input("Enter command: ").strip().lower()

            if not command_input:
                continue

            if (
                command_input == "quit"
                or command_input == "exit"
                or command_input == "q"
            ):
                print("\n✓ Exiting...")
                break

            elif command_input.startswith("takeoff") or command_input == "1":
                if command_input == "1":
                    altitude_input: str = input("Enter altitude in meters (default: 5.0): ").strip()
                    if altitude_input:
                        try:
                            altitude: float = float(altitude_input)
                        except ValueError:
                            print("✗ Invalid altitude. Using default (5.0m)")
                            altitude = 5.0
                    else:
                        altitude = 5.0
                else:
                    parts = command_input.split()
                    if len(parts) > 1:
                        try:
                            altitude = float(parts[1])
                        except ValueError:
                            print("✗ Invalid altitude. Using default (5.0m)")
                            altitude = 5.0
                    else:
                        altitude = 5.0
                
                send_takeoff(client, altitude)

            elif command_input == "land" or command_input == "2":
                send_land(client)

            elif command_input.startswith("move") or command_input == "3":
                if command_input == "3":
                    destination: str = input("Enter destination: ").strip()
                else:
                    parts = command_input.split(maxsplit=1)
                    if len(parts) < 2:
                        print("✗ Usage: move <destination>")
                        continue
                    destination = parts[1]

                send_move(client, destination)

            elif command_input == "auto_land" or command_input == "4":
                if aruco_detection_data and aruco_detection_data.get("detected", False):
                    print("→ Initiating auto-land on ArUco marker...")
                    send_auto_land(client)
                else:
                    print("✗ No ArUco marker detected. Cannot auto-land.")

            elif command_input == "enable_centering" or command_input == "5":
                print("→ Enabling centering mode...")
                send_enable_centering(client)

            elif command_input == "disable_centering" or command_input == "6":
                print("→ Disabling centering mode...")
                send_disable_centering(client)

            elif command_input == "video" or command_input == "7":
                if not video_window_active:
                    print("→ Opening video feed window...")
                    video_thread = Thread(target=display_video_stream, daemon=True)
                    video_thread.start()
                else:
                    print("ℹ Video window is already active")

            elif command_input == "custom" or command_input == "8":
                print(
                    '\nEnter custom JSON command (e.g., {"action": "test", "value": "123"}):'
                )
                json_input: str = input("> ").strip()

                try:
                    parsed_json: Dict = json.loads(json_input)

                    print(f"\n→ Sending custom command: {json_input}")
                    result = client.publish(COMMAND_TOPIC, json_input, qos=1)
                    result.wait_for_publish()

                    if result.is_published():
                        print("✓ Custom command sent")
                    else:
                        print("✗ Failed to send custom command")

                except json.JSONDecodeError as e:
                    print(f"✗ Invalid JSON: {e}")

            else:
                print(f"✗ Unknown command: {command_input}")
                print("Type 'quit' to exit or enter a valid command (1-9)")

        except KeyboardInterrupt:
            print("\n\n✓ Interrupted by user. Exiting...")
            break
        except Exception as e:
            print(f"✗ Error: {e}")


def main() -> None:
    client: mqtt.Client = mqtt.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_message = on_message

    try:
        print(f"Connecting to MQTT broker at {BROKER_ADDRESS}:{BROKER_PORT}...")
        client.connect(BROKER_ADDRESS, BROKER_PORT, 60)

        client.loop_start()

        time.sleep(1)

        if len(sys.argv) > 1:
            action: str = sys.argv[1].lower()

            if action == "takeoff":
                altitude: float = 5.0
                if len(sys.argv) > 2:
                    try:
                        altitude = float(sys.argv[2])
                    except ValueError:
                        print(f"✗ Invalid altitude: {sys.argv[2]}. Using default (5.0m)")
                send_takeoff(client, altitude)
            elif action == "land":
                send_land(client)
            elif action == "move":
                if len(sys.argv) < 3:
                    print("✗ Usage: python sender.py move <destination>")
                    return
                destination: str = sys.argv[2]
                send_move(client, destination)
            elif action == "auto_land":
                send_auto_land(client)
            elif action == "enable_centering":
                send_enable_centering(client)
            elif action == "disable_centering":
                send_disable_centering(client)
            else:
                print(f"✗ Unknown action: {action}")
                print("Valid actions: takeoff [altitude], land, move <destination>, auto_land, enable_centering, disable_centering")
                return

            time.sleep(1)
        else:
            interactive_mode(client)

    except ConnectionRefusedError:
        print(f"✗ Could not connect to MQTT broker at {BROKER_ADDRESS}:{BROKER_PORT}")
        print("  Make sure the broker is running (e.g., mosquitto)")
    except Exception as e:
        print(f"✗ Error: {e}")
    finally:
        client.loop_stop()
        client.disconnect()
        print("✓ Disconnected from MQTT broker")


if __name__ == "__main__":
    main()
