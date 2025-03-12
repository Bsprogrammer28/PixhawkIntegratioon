from pymavlink import mavutil
import time

CONNECTION_STRING = 'COM6'  # Change to your Pixhawk port
BAUD_RATE = 9600  # Default for Pixhawk

print("Connecting to Pixhawk...")
connection = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
connection.wait_heartbeat()
print(f"Connected to Pixhawk (System {connection.target_system}, Component {connection.target_component})")

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

set_mode("STABILIZE")
time.sleep(2)

def arm_motors():
    print("Arming motors...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    connection.motors_armed_wait()
    print("Motors Armed!")

arm_motors()
time.sleep(2)

def set_motor_speed(motor_channel, pwm=1750):
    pwm_values = [1000] * 8  # Initialize all motors to 1000 PWM (stopped)
    pwm_values[motor_channel - 1] = pwm  # Set the target motor's PWM
    connection.mav.servo_output_raw_send(
        0,  # Time usec (not needed, set to 0)
        1,  # Port (typically 1 for most Pixhawk setups)
        *pwm_values  # Expand the list to provide all 8 PWM values
    )
    print(f"Motor {motor_channel} set to {pwm} PWM")

def stop_motors():
    for motor in range(1, 5):  # Adjust according to motor mapping
        set_motor_speed(motor, 1000)
    print("All motors stopped.")

time.sleep(2)

motor_channels = [1, 2, 3, 4]  # Adjust based on your Pixhawk motor configuration

for motor in motor_channels:
    print(f"Testing Motor {motor}...")
    set_motor_speed(motor, 1750)
    time.sleep(3)
    set_motor_speed(motor, 1000)
    time.sleep(2)

def disarm_motors():
    print("Disarming motors...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Motors Disarmed!")

disarm_motors()
time.sleep(2)
print("Motor Test Complete.")
