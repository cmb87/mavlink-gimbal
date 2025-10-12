import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# Create UDP connection
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14565', source_system=1, source_component=154)  # 191 = MAV_COMP_ID_GIMBAL

# Gimbal state
pitch = 0.0  # degrees
yaw = 90.0    # degrees
roll = 0.0   # degrees

def euler_to_quaternion(yaw, pitch, roll):
    """Convert Euler angles (in degrees) to quaternion (w, x, y, z)."""
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    
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

def quaternion_to_euler(q):
    """Convert quaternion (w, x, y, z) to Euler angles (yaw, pitch, roll) in degrees."""
    w, x, y, z = q
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    
    return yaw, pitch, roll

def send_heartbeat():
    """Send heartbeat message to indicate gimbal is active."""
    connection.mav.heartbeat_send(
        type=mavlink2.MAV_TYPE_GIMBAL,
        autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavlink2.MAV_STATE_ACTIVE
    )

def send_gimbal_device_information():
    """Send gimbal device information to advertise capabilities."""
    connection.mav.gimbal_device_information_send(
        time_boot_ms=int(time.time() * 1000) % (2**32),
        vendor_name=b'xAI',
        model_name=b'SimGimbal',
        custom_name=b'SimGimbal',
        firmware_version=0,
        hardware_version=0,  # Version
        uid=0,
        cap_flags=mavlink2.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS| mavlink2.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS | mavlink2.GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL,
        custom_cap_flags=0,
        roll_min=0.0,
        roll_max=0.0,
        pitch_min=-90.0*3.145/180,
        pitch_max=0.0,
        yaw_min=-90.0*3.145/180,
        yaw_max=90.0*3.145/180,
        gimbal_device_id=154,
    )

def send_gimbal_attitude():
    """Send gimbal attitude status."""

    q = euler_to_quaternion(yaw, pitch, roll)
    
    print(yaw)
    connection.mav.gimbal_device_attitude_status_send(
        target_system=0,
        target_component=0,
        time_boot_ms=int(time.time() * 1000) % (2**32),
        flags=0,
        q=q,
        angular_velocity_x=0.0,
        angular_velocity_y=0.0,
        angular_velocity_z=0.0,
        failure_flags=0
    )

def send_command_ack(command, result):
    """Send COMMAND_ACK for received commands."""
    connection.mav.command_ack_send(
        command=command,
        result=result  # e.g., mavlink2.MAV_RESULT_ACCEPTED
    )


def handle_gimbal_device_set_attitude(msg):
    """Handle incoming gimbal attitude commands"""
    global pitch, yaw, roll
    # Extract commanded angles (in radians)
    # Quaternion, but we'll use Euler angles for simplicity

    (yaw, pitch, roll) = quaternion_to_euler(msg.q)

    

    print(f"Received attitude command: Pitch={pitch:.2f}°, Yaw={yaw:.2f}°, Roll={roll:.2f}°")

def handle_gimbal_device_set_pitchyaw(msg):
    """Handle GIMBAL_DEVICE_SET_PITCHYAW from manager."""
    global pitch, yaw
    pitch_rad = msg.pitch  # Radians
    yaw_rad = msg.yaw      # Radians
    pitch = math.degrees(pitch_rad)
    yaw = math.degrees(yaw_rad)
    
    # Handle flags for frame (earth/body) and type (angle/rate) - simplified
    flags = msg.flags
    if flags & mavlink2.GIMBAL_FLAGS_YAW_LOCK:
        print("Yaw lock enabled")
    if flags & mavlink2.GIMBAL_FLAGS_PITCH_LOCK:
        print("Pitch lock enabled")
    
    print(f"Received pitch/yaw set: Pitch={pitch:.2f}°, Yaw={yaw:.2f}° (Flags: {flags})")



def handle_command_long(msg):
    """Handle COMMAND_LONG messages."""
    command = msg.command
    if command == mavlink2.MAV_CMD_REQUEST_MESSAGE:
        message_id = int(msg.param1)
        if message_id == mavlink2.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
            send_gimbal_device_information()
            send_command_ack(command, mavlink2.MAV_RESULT_ACCEPTED)
            print(f"Handled request for GIMBAL_DEVICE_INFORMATION (ID: {message_id})")
        else:
            send_command_ack(command, mavlink2.MAV_RESULT_UNSUPPORTED)
            print(f"Unsupported message ID requested: {message_id}")
    elif command == mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
        send_command_ack(command, mavlink2.MAV_RESULT_ACCEPTED)
        print("Gimbal manager control acknowledged")
    else:
        send_command_ack(command, mavlink2.MAV_RESULT_UNSUPPORTED)
        print(f"Unsupported command: {command}")

        


def main():
    print("Starting MAVLink gimbal device on udpin:0.0.0.0:14565")
    
    # Wait for initial heartbeat from GCS/manager
    connection.wait_heartbeat()
    print("Connected to MAVLink system")
    
    # Send device info immediately
   # send_gimbal_device_information()
    
    last_heartbeat = 0
    last_attitude = 0
    last_info = time.time()  # Send info every 30s
    
    while True:
        current_time = time.time()
        
        # Send heartbeat every 1s
        if current_time - last_heartbeat >= 1.0:
            send_heartbeat()
            last_heartbeat = current_time
        

        # Send attitude every 0.1s
        if current_time - last_attitude >= 0.1:
            send_gimbal_attitude()
            last_attitude = current_time
        
        # Handle incoming messages
        msg = connection.recv_match(blocking=False)
        if msg is not None:
            msg_type = msg.get_type()

           # print(msg_type if "GIMAL" in msg_type else "")

            if msg_type == 'GIMBAL_DEVICE_SET_ATTITUDE':
              #  print("Set attitude")
                handle_gimbal_device_set_attitude(msg)
            elif msg_type == 'GIMBAL_DEVICE_SET_PITCHYAW':
                print("Set pitch yaw")
                handle_gimbal_device_set_pitchyaw(msg)
            elif msg_type == 'COMMAND_LONG':
                print("Command")
                handle_command_long(msg)
            # Add more handlers as needed (e.g., GIMBAL_MANAGER_SET_MANUAL_CONTROL)
        
        time.sleep(0.005)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down gimbal device")
        connection.close()