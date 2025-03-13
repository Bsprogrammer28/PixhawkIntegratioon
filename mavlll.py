from pymavlink import mavutil
import time

# Connect to Pixhawk via Telemetry (Change COM Port if needed)
CONNECTION_STRING = 'COM7'  # Replace with your telemetry port (e.g., 'COM7' or '/dev/ttyUSB0')
BAUD_RATE = 57600  # Standard telemetry baud rate

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

# Disable pre-arm checks (Since GPS is not available)
def disable_prearm_checks():
    print("Disabling pre-arm checks...")
    connection.mav.param_set_send(
        connection.target_system, connection.target_component,
        b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1)
    print("Pre-arm checks disabled.")

# Arm the motors
def arm_motors():
    print("Arming motors...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    connection.motors_armed_wait()
    print("Motors Armed!")

# Function to maintain altitude using local position setpoint
def maintain_altitude(target_altitude, duration=5):
    print(f"Holding altitude at {target_altitude}m for {duration} seconds...")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not required)
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Only control Z (altitude)
            0, 0, -target_altitude,  # X, Y, Z (NED frame, so Z is negative for altitude)
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0, 0  # yaw, yaw_rate
        )
        time.sleep(0.2)  # Send command every 200ms

# Landing function
def land():
    print("Landing...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(10)  # Allow time for landing
    print("Landed successfully!")

# Execute flight sequence
set_mode("ALT_HOLD")  # Use Altitude Hold mode
time.sleep(2)
disable_prearm_checks()
time.sleep(2)
arm_motors()
time.sleep(2)
maintain_altitude(target_altitude=1, duration=5)  # Maintain altitude at 2m for 5 sec
land()
