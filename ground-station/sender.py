import paho.mqtt.client as mqtt
import json
import time
import sys
import cv2
import numpy as np
import base64
from typing import Dict, Optional
from threading import Thread
import queue


# --- Configuration Mode ---
# Set to True for simulation, False for real drone
SIMULATION_MODE: bool = False

# MQTT Broker Configuration
# For simulation: use localhost
# For real drone: use drone's IP address (e.g., "192.168.0.163")
BROKER_ADDRESS: str = "localhost" if SIMULATION_MODE else "192.168.0.231"
BROKER_PORT: int = 1883
COMMAND_TOPIC: str = "drone/commands"
STREAM_TOPIC: str = "camera/stream"
ARUCO_TOPIC: str = "drone/aruco_detection"

latest_frame: Optional[np.ndarray] = None
aruco_detection_data: Optional[Dict] = None
video_window_active: bool = False
command_queue: queue.Queue = queue.Queue()
running: bool = True
frame_counter: int = 0
first_frame_received: bool = False
last_display_time: float = 0.0  


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
    global latest_frame, aruco_detection_data, frame_counter, first_frame_received
    
    try:
        if msg.topic == STREAM_TOPIC:
            jpg_as_text: str = msg.payload.decode("utf-8")
            jpg_original: bytes = base64.b64decode(jpg_as_text)
            jpg_as_np: np.ndarray = np.frombuffer(jpg_original, dtype=np.uint8)
            latest_frame = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
            
            if latest_frame is not None:
                frame_counter += 1
                if not first_frame_received:
                    print("✓ Video feed connected")
                    first_frame_received = True
            
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
    return send_command(client, "takeoff", altitude=altitude)


def send_land(client: mqtt.Client) -> bool:
    return send_command(client, "land")


def send_move(client: mqtt.Client, destination: str) -> bool:
    return send_command(client, "move", destination)


def send_auto_land(client: mqtt.Client) -> bool:
    return send_command(client, "auto_land")


def send_enable_centering(client: mqtt.Client) -> bool:
    return send_command(client, "enable_centering")


def send_disable_centering(client: mqtt.Client) -> bool:
    return send_command(client, "disable_centering")


def start_video_window() -> None:
    global video_window_active
    if not video_window_active:
        try:
            cv2.namedWindow("Drone Camera Feed", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Drone Camera Feed", 800, 600)
            video_window_active = True
            print("✓ Video window opened")
        except Exception as e:
            print(f"✗ Error opening video window: {e}")
            video_window_active = False


def update_video_display() -> None:
    global latest_frame, last_display_time
    
    if latest_frame is not None and video_window_active:
        try:
            current_time: float = time.time()
            if current_time - last_display_time >= 0.016:  # ~60 FPS
                cv2.imshow("Drone Camera Feed", latest_frame)
                cv2.waitKey(1)
                last_display_time = current_time
        except Exception:
            pass 


def close_video_window() -> None:
    global video_window_active
    if video_window_active:
        cv2.destroyAllWindows()
        video_window_active = False
        print("✓ Video window closed")


def input_thread_func() -> None:
    global running
    while running:
        try:
            user_input: str = input("Enter command: ").strip().lower()
            if user_input:
                command_queue.put(user_input)
        except EOFError:
            break
        except Exception as e:
            print(f"✗ Input error: {e}")
            break


def interactive_mode(client: mqtt.Client) -> None:
    global video_window_active, running
    
    print("\n" + "=" * 60)
    print("LACC DRONE | CLI")
    print("=" * 60)
    print("\nMode: {}".format("SIMULATION" if SIMULATION_MODE else "REAL DRONE"))
    print("Broker: {}:{}".format(BROKER_ADDRESS, BROKER_PORT))
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
    print("  status                 - Show video feed and connection status")
    print("  9. quit                - Exit the program")
    print("\n" + "=" * 60 + "\n")
    
    start_video_window()
    
    input_thread: Thread = Thread(target=input_thread_func, daemon=True)
    input_thread.start()

    while running:
        update_video_display()
        
        try:
            command_input: str = command_queue.get_nowait()
        except queue.Empty:
            time.sleep(0.0001)  # 0.1ms delay for better responsiveness
            continue

        try:
            if not command_input:
                continue

            if (
                command_input == "quit"
                or command_input == "exit"
                or command_input == "q"
            ):
                print("\n✓ Exiting...")
                running = False
                close_video_window()
                break

            elif command_input.startswith("takeoff") or command_input == "1":
                altitude: float = 5.0
                if command_input == "1":
                    print("Using default altitude: 5.0m (type 'takeoff <altitude>' to specify)")
                else:
                    parts = command_input.split()
                    if len(parts) > 1:
                        try:
                            altitude = float(parts[1])
                        except ValueError:
                            print("✗ Invalid altitude. Using default (5.0m)")
                            altitude = 5.0
                
                send_takeoff(client, altitude)

            elif command_input == "land" or command_input == "2":
                send_land(client)

            elif command_input.startswith("move") or command_input == "3":
                if command_input == "3":
                    print("✗ Usage: move <destination>")
                    continue
                else:
                    parts = command_input.split(maxsplit=1)
                    if len(parts) < 2:
                        print("✗ Usage: move <destination>")
                        continue
                    destination: str = parts[1]

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
                    start_video_window()
                else:
                    print("→ Closing video feed window...")
                    close_video_window()
            
            elif command_input == "status":
                print(f"\n📊 Status:")
                print(f"  Video window active: {video_window_active}")
                print(f"  Frames received: {frame_counter}")
                print(f"  Latest frame: {'Available' if latest_frame is not None else 'None'}")
                if latest_frame is not None:
                    height, width = latest_frame.shape[:2]
                    print(f"  Frame size: {width}x{height}")
                print(f"  ArUco markers detected: {aruco_detection_data.get('detected', False) if aruco_detection_data else False}")
                print()

            elif command_input.startswith("custom") or command_input == "8":
                if command_input == "8" or command_input == "custom":
                    print('✗ Usage: custom {"action": "test", "value": "123"}')
                    continue
                
                json_start: int = command_input.find("{")
                if json_start == -1:
                    print('✗ Usage: custom {"action": "test", "value": "123"}')
                    continue
                    
                json_input: str = command_input[json_start:].strip()

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
            running = False
            close_video_window()
            break
        except Exception as e:
            print(f"✗ Error: {e}")
    
    close_video_window()


def main() -> None:
    client: mqtt.Client = mqtt.Client()
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_message = on_message
    
    client.max_queued_messages_set(1) 
    client.max_inflight_messages_set(1) 

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
