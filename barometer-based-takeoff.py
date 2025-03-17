from pymavlink import mavutil
import time

def barometer_takeoff(connection_string, target_altitude):
    """
    Execute a barometer-based takeoff and altitude hold with Pixhawk.
    
    Args:
        connection_string: MAVLink connection string (e.g., 'udpin:0.0.0.0:14550')
        target_altitude: Target altitude in meters
    """
    print(f"Connecting to vehicle on: {connection_string}")
    
    # Connect to the Vehicle
    master = mavutil.mavlink_connection(connection_string)
    
    # Wait for the first heartbeat
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Heartbeat received!")
    
    # Set the vehicle to STABILIZE mode first
    print("Setting STABILIZE mode...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0  # Stabilize mode code for ArduPilot
    )
    
    # Wait for mode change acknowledgment
    wait_for_mode(master, "STABILIZE")
    
    # Arm the vehicle
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # ARM
        0, 0, 0, 0, 0, 0  # Other parameters (not used)
    )
    
    # Wait for arming
    wait_for_arm(master)
    print("Vehicle armed!")
    
    # Take off in STABILIZE mode using RC override
    print(f"Taking off to {target_altitude} meters...")
    initial_alt = get_barometer_altitude(master)
    print(f"Initial altitude: {initial_alt} meters")
    
    # Use throttle RC override to ascend
    throttle_value = 1600  # Medium-high throttle (around 60-70%)
    
    while True:
        # Send throttle command
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, throttle_value, 0, 0, 0, 0, 0  # Only override throttle (channel 3)
        )
        
        # Check current altitude
        current_alt = get_barometer_altitude(master)
        alt_diff = current_alt - initial_alt
        print(f"Current altitude: {current_alt}m, Gained: {alt_diff}m")
        
        # If we've reached target altitude, break the loop
        if alt_diff >= target_altitude:
            print(f"Reached target altitude of {target_altitude}m")
            break
            
        # Small delay to prevent flooding the connection
        time.sleep(0.5)
    
    # Stop the throttle override
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0  # Release all overrides
    )
    
    # Switch to ALT_HOLD mode to maintain altitude using barometer
    print("Switching to ALT_HOLD mode...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        2  # ALT_HOLD mode code for ArduPilot
    )
    
    # Wait for mode change acknowledgment
    wait_for_mode(master, "ALT_HOLD")
    print("Now in ALT_HOLD mode, maintaining altitude with barometer")
    
    # Monitor altitude and handle emergencies
    monitor_altitude(master, target_altitude, initial_alt)


def get_barometer_altitude(master):
    """Get the current barometer altitude in meters."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        return msg.relative_alt / 1000.0  # Convert mm to meters
    return 0


def wait_for_arm(master, timeout=30):
    """Wait for the vehicle to arm."""
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
        time.sleep(0.1)
    raise Exception("Vehicle failed to arm within timeout")


def wait_for_mode(master, mode_name, timeout=30):
    """Wait for the vehicle to enter the specified mode."""
    # Dict mapping mode names to their numbers for ArduCopter
    mode_map = {
        "STABILIZE": 0,
        "ALT_HOLD": 2,
        "LOITER": 5,
        "RTL": 6,
        "AUTO": 3,
        "LAND": 9
    }
    
    target_mode = mode_map.get(mode_name)
    if target_mode is None:
        raise ValueError(f"Unknown mode: {mode_name}")
    
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == target_mode:
            print(f"Mode changed to {mode_name}")
            return True
        time.sleep(0.1)
    raise Exception(f"Failed to enter {mode_name} mode within timeout")


def monitor_altitude(master, target_altitude, initial_altitude, drop_threshold=2.0):
    """
    Monitor altitude and handle emergencies.
    
    Args:
        master: MAVLink connection
        target_altitude: Target altitude in meters
        initial_altitude: Initial altitude in meters
        drop_threshold: Altitude drop threshold in meters to trigger emergency
    """
    print("Monitoring altitude...")
    
    while True:
        # Get current altitude
        current_alt = get_barometer_altitude(master)
        alt_diff = current_alt - initial_altitude
        
        print(f"Current altitude: {current_alt}m, Target: {target_altitude}m, Diff: {alt_diff}m")
        
        # Check for significant altitude drop
        if alt_diff < target_altitude - drop_threshold:
            print(f"WARNING: Altitude dropped by {drop_threshold}m or more!")
            print("Initiating emergency disarm...")
            
            # Emergency disarm
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                0,  # DISARM
                0, 0, 0, 0, 0, 0  # Other parameters (not used)
            )
            
            print("Vehicle disarmed due to altitude drop")
            break
        
        # Wait before next check
        time.sleep(1)


if __name__ == "__main__":
    # Example usage
    CONNECTION_STRING = 'udpin:0.0.0.0:14550'  # Update with your connection string
    TARGET_ALTITUDE = 5  # Target altitude in meters
    
    barometer_takeoff(CONNECTION_STRING, TARGET_ALTITUDE)
