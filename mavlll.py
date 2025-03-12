from pymavlink import mavutil
import time

# Use telemetry port (Find in Device Manager, e.g., 'COM7' for Windows or '/dev/ttyUSB0' for Linux)
CONNECTION_STRING = 'COM7'  # Change this to your telemetry port
BAUD_RATE = 57600  # Typical baud rate for telemetry

# Connect to Pixhawk via telemetry
print("Connecting to Pixhawk via Telemetry...")
connection = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
connection.wait_heartbeat()
print(f"Connected to Pixhawk (System {connection.target_system}, Component {connection.target_component})")

# Function to set flight mode
def set_mode(mode):
    if mode not in connection.mode_mapping():
        print(f"Mode {mode} not available.")
        return
    mode_id = connection.mode_mapping()[mode]
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Switched to {mode} mode")

def disable_prearm_checks():
    print("Disabling pre-arm checks...")
    connection.mav.param_set_send(
        connection.target_system, connection.target_component,
        b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1)  # Allow time for the parameter change
    print("Pre-arm checks disabled.")

def arm_motors():
    print("Arming motors...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    connection.motors_armed_wait()
    print("Motors Armed!")

def set_throttle(throttle):
    """ Set the throttle manually using RC override. """
    connection.mav.rc_channels_override_send(
        connection.target_system, connection.target_component,
        0, 0, throttle, 0, 0, 0, 0, 0  # Channel 3 is throttle
    )
    print(f"Throttle set to {throttle}")

def takeoff():
    print("Taking off...")
    set_throttle(1600)  # Increase throttle to take off
    time.sleep(5)  # Hover for 5 seconds

def land():
    print("Landing...")
    set_throttle(1300)  # Gradually reduce throttle
    time.sleep(3)
    set_throttle(1000)  # Fully lower throttle for landing
    print("Landed successfully!")

# Execute flight sequence
set_mode("ALT_HOLD")
time.sleep(2)
disable_prearm_checks()
time.sleep(2)
arm_motors()
time.sleep(2)
takeoff()
land()
