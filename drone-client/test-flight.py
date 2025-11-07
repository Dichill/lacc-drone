import time
from pymavlink import mavutil
from logger import get_logger, init_logger, close_logger

master = mavutil.mavlink_connection("/dev/serial0", baud=57600)


def wait_for_ready(master) -> None:
    logger = get_logger()
    logger.info("Waiting for heartbeat...")
    master.wait_heartbeat()
    logger.event("HEARTBEAT_RECEIVED", 
                system=master.target_system, 
                component=master.target_component)
    logger.info(f"Heartbeat from system (system {master.target_system} component {master.target_component})",
               system=master.target_system, 
               component=master.target_component)
    time.sleep(2)


def arm_and_takeoff(master, target_altitude: float) -> None:
    logger = get_logger()
    
    if master.motors_armed():
        logger.info("Vehicle is already armed")
    else:
        logger.info("Waiting for arming...")
        logger.event("ARMING_MOTORS")
        master.arducopter_arm()
        time.sleep(2)
        logger.event("MOTORS_ARMED")

    logger.info("Setting flight mode to GUIDED...")
    master.set_mode("GUIDED")
    logger.event("FLIGHT_MODE_CHANGED", mode="GUIDED")
    time.sleep(1)

    logger.info(f"Taking off to {target_altitude} meters...", target_altitude=target_altitude)
    logger.event("TAKEOFF_INITIATED", altitude=target_altitude)
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


def wait_for_altitude(master, target_altitude: float, tolerance: float = 0.5) -> None:
    logger = get_logger()
    logger.info("Waiting for altitude...", target_altitude=target_altitude, tolerance=tolerance)
    start_time: float = time.time()
    
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if not msg:
            continue

        current_altitude: float = msg.relative_alt / 1000.0
        logger.metric("current_altitude", current_altitude, "meters")
        logger.info(f"Altitude: {current_altitude:.2f}m", current_altitude=current_altitude)

        if abs(current_altitude - target_altitude) < tolerance:
            logger.event("TARGET_ALTITUDE_REACHED", 
                        target=target_altitude, 
                        current=current_altitude)
            logger.info("Target altitude reached")
            break

        elapsed_time: float = time.time() - start_time
        if elapsed_time > 30:
            logger.warning("Timed out waiting for altitude", 
                          elapsed_time=elapsed_time,
                          current_altitude=current_altitude)
            break

        time.sleep(1)


def land(master) -> None:
    logger = get_logger()
    logger.event("LANDING_INITIATED")
    logger.info("Initiating landing sequence...")
    master.set_mode("LAND")
    logger.event("FLIGHT_MODE_CHANGED", mode="LAND")
    time.sleep(1)

    while master.motors_armed():
        logger.info("Waiting for vehicle to disarm...")
        time.sleep(1)

    logger.event("LANDING_COMPLETE")
    logger.info("Landing complete")


if __name__ == "__main__":
    logger = init_logger()
    logger.event("TEST_FLIGHT_START")
    logger.info("Test flight script starting")
    
    try:
        wait_for_ready(master)

        target_altitude: float = 5.0
        logger.info(f"Target altitude set to {target_altitude} meters", target_altitude=target_altitude)
        
        arm_and_takeoff(master, target_altitude)
        wait_for_altitude(master, target_altitude)

        hold_duration: int = 10
        logger.info(f"Holding position for {hold_duration} seconds...", hold_duration=hold_duration)
        logger.event("POSITION_HOLD_START", duration=hold_duration)
        time.sleep(hold_duration)
        logger.event("POSITION_HOLD_END")

        land(master)

    except Exception as e:
        logger.error(f"An error occurred: {e}", error=str(e), error_type=type(e).__name__)
    finally:
        logger.event("TEST_FLIGHT_END")
        logger.info("Script finished")
        master.close()
        close_logger()
