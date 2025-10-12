import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


def quaternion_to_euler(q):
    """Convert quaternion (w, x, y, z) to Euler angles (yaw, pitch, roll) in degrees.
    
    Args:
        q (list): Quaternion as [w, x, y, z]
    
    Returns:
        tuple: (yaw, pitch, roll) in degrees
    """
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Handle singularity at ±90 degrees
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert radians to degrees
    yaw = math.degrees(yaw)
    pitch = math.degrees(pitch)
    roll = math.degrees(roll)
    
    return yaw, pitch, roll

def euler_to_quaternion(yaw, pitch, roll):
    """Convert Euler angles (in degrees) to quaternion"""
    # Convert degrees to radians
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    
    # Calculate quaternion components
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [w, x, y, z]


# Create UDP connection
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14565', source_system=1, source_component=154)  # 191 is typical for gimbal

# Gimbal state
pitch = 0.0  # degrees
yaw = 30.0    # degrees
roll = 0.0   # degrees

def send_heartbeat():
    """Send heartbeat message to indicate gimbal is active"""
    connection.mav.heartbeat_send(
        type=mavlink2.MAV_TYPE_GIMBAL,
        autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavlink2.MAV_STATE_ACTIVE
    )

def send_gimbal_attitude():
    """Send gimbal attitude status"""
    # Convert Euler angles to quaternion
    q = euler_to_quaternion(yaw, pitch, roll)
    
    print(yaw)
    connection.mav.gimbal_device_attitude_status_send(
        target_system=1,  # Broadcast to all
        target_component=1,
        time_boot_ms=int(time.time() * 1000) % (2**32),  # Milliseconds since boot
        flags=0,
        q=q,  # Quaternion [w, x, y, z]
        angular_velocity_x=0.0,
        angular_velocity_y=0.0,
        angular_velocity_z=0.0,
        failure_flags=0  # No failures
    )


def handle_gimbal_device_set_attitude(msg):
    """Handle incoming gimbal attitude commands"""
    global pitch, yaw, roll
    # Extract commanded angles (in radians)
    # Quaternion, but we'll use Euler angles for simplicity

    (yaw_rad, pitch_rad, roll_rad) = quaternion_to_euler(msg.q)

    
    # Convert to degrees for internal state
    pitch = math.degrees(pitch_rad)
    #yaw = math.degrees(yaw_rad)
    roll = math.degrees(roll_rad)
    
    print(f"Received attitude command: Pitch={pitch:.2f}°, Yaw={yaw:.2f}°, Roll={roll:.2f}°")

def main():
    print("Starting MAVLink gimbal device on udpin:0.0.0.0:14565")
    
    # Wait for connection
    connection.wait_heartbeat()
    print("Connected to MAVLink system")
    
    last_heartbeat = 0
    last_attitude = 0
    
    while True:
        # Send heartbeat every 1 second
        current_time = time.time()
        if current_time - last_heartbeat >= 1.0:
            send_heartbeat()
            last_heartbeat = current_time
        
        # Send attitude update every 0.1 seconds
        if current_time - last_attitude >= 0.1:
            send_gimbal_attitude()
            last_attitude = current_time
        
        # Handle incoming messages
        msg = connection.recv_match(blocking=False)
        if msg is not None:
            if msg.get_type() == 'GIMBAL_DEVICE_SET_ATTITUDE':
               
                handle_gimbal_device_set_attitude(msg)
        
        time.sleep(0.01)  # Prevent CPU overload

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down gimbal device")
        connection.close()