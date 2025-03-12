from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
# from pymavlink import mavutil

# Replace COM3 with your actual Pixhawk COM port
vehicle = connect('COM6', baud=9600, wait_ready=True)

vehicle.parameters['ARMING_CHECK'] = 0
vehicle.parameters['GPS_TYPE'] = 0  # Disables GPS

vehicle.mode = VehicleMode("STABILIZE")

while not vehicle.mode.name == "STABILIZE":
    print("Waiting for mode change...")
    time.sleep(1)

print("Mode set to STABILIZE")

print("Arming motors...")
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

print("Motors armed!")

def test_motor(channel, pwm):
    print(f"Testing Motor {channel} at PWM {pwm}")
    vehicle.channels.overrides[str(channel)] = pwm
    time.sleep(3)  # Run for 3 seconds
    vehicle.channels.overrides[str(channel)] = 1100  # Stop motor
    time.sleep(1)

# Run motors one by one
for i in range(1, 5):  # Change range for different drones (1-4 for quadcopter)
    test_motor(i, 2600)

print("Motor test complete!")

# master = mavutil.mavlink_connection('COM6', baud=9600)
# master.wait_heartbeat()

# Disarm
vehicle.armed = False
while vehicle.armed:
    print("Waiting to disarm...")
    time.sleep(1)

print("Motors disarmed!")
vehicle.close()

