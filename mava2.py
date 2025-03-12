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
