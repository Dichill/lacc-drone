import paho.mqtt.client as mqtt
import json
import time

"""

    MAVLINK

"""
# from pymavlink import mavutil

# --- Connection to Pixhawk via serial port ---
# On a Raspberry Pi, the serial port is typically /dev/serial0
# The baud rate must match the SERIAL2_BAUD setting on your Pixhawk.
# master = mavutil.mavlink_connection("/dev/serial0", baud=57600)
broker_address = "192.168.0.163"
broker_port = 1883
command_topic = "drone/commands"


def process_command(command):
    """Parses a command and takes action on the drone."""
    print(f"Received command: {command}")
    try:
        data = json.loads(command)
        action = data.get("action")
        value = data.get("value")

        if action == "takeoff":
            print("Action: Executing 'takeoff' command.")
        elif action == "land":
            print("Action: Executing 'land' command.")
        elif action == "set_speed":
            print(f"Action: Setting speed to {value}.")
        else:
            print(f"Unknown command action: {action}")

    except json.JSONDecodeError:
        print("Received non-JSON message, ignoring.")


def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    print(f"Connected to MQTT Broker with result code {rc}")
    client.subscribe(command_topic)
    print(f"Subscribed to topic: {command_topic}")


def on_message(client, userdata, msg):
    """Callback for when a message is received on a subscribed topic."""
    payload = msg.payload.decode()
    print(f"[{msg.topic}] {payload}")
    process_command(payload)


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(broker_address, broker_port, 60)
    except Exception as e:
        print(f"Could not connect to broker: {e}")
        return

    client.loop_forever()


if __name__ == "__main__":
    main()
