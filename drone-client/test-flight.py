import time
from pymavlink import mavutil

# --- Connection to Pixhawk via serial port ---
# On a Raspberry Pi, the serial port is typically /dev/serial0
# The baud rate must match the SERIAL2_BAUD setting on your Pixhawk.
master = mavutil.mavlink_connection("/dev/serial0", baud=57600)


def wait_for_ready(master):
    """Waits for a heartbeat from the vehicle to ensure a connection exists."""
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(
        "Heartbeat from system (system %u component %u)"
        % (master.target_system, master.target_component)
    )
    time.sleep(2)


def arm_and_takeoff(master, target_altitude):
    """Arms the motors and commands a takeoff to a specified altitude."""
    if master.motors_armed():
        print("Vehicle is already armed.")
    else:
        print("Waiting for arming...")
        master.arducopter_arm()
        time.sleep(2)

    print("Setting flight mode to GUIDED...")
    master.set_mode("GUIDED")
    time.sleep(1)

    print(f"Taking off to {target_altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        target_altitude,
    )


def wait_for_altitude(master, target_altitude, tolerance=0.5):
    """Waits until the vehicle reaches the target altitude."""
    print("Waiting for altitude...")
    start_time = time.time()
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if not msg:
            continue

        current_altitude = msg.relative_alt / 1000.0
        print(f"Altitude: {current_altitude:.2f}m")

        if abs(current_altitude - target_altitude) < tolerance:
            print("Target altitude reached.")
            break

        if time.time() - start_time > 30:
            print("Timed out waiting for altitude.")
            break

        time.sleep(1)


def land(master):
    """Changes the flight mode to LAND to initiate an automated landing."""
    print("Initiating landing sequence...")
    master.set_mode("LAND")
    time.sleep(1)

    while master.motors_armed():
        print("Waiting for vehicle to disarm...")
        time.sleep(1)

    print("Landing complete.")


if __name__ == "__main__":
    try:
        wait_for_ready(master)

        target_altitude = 5
        arm_and_takeoff(master, target_altitude)

        wait_for_altitude(master, target_altitude)

        print("Holding position for 10 seconds...")
        time.sleep(10)

        land(master)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Script finished.")
        master.close()
