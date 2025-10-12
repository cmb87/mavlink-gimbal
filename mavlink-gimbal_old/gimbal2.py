import time
from pymavlink import mavutil

# Connection string: Adjust to your setup (e.g., 'udpout:127.0.0.1:14550' or 'udpin:0.0.0.0:14551')
connection_string = 'udpin:0.0.0.0:14590'

# System and component ID for the mock gimbal
sys_id = 1  # Same as vehicle
comp_id = mavutil.mavlink.MAV_COMP_ID_GIMBAL

print(f"Starting mock MAVLink gimbal on {connection_string}")

# Establish the MAVLink connection
conn = mavutil.mavlink_connection(connection_string, source_system=sys_id, source_component=comp_id)

# Wait for a heartbeat from the connected system (e.g., autopilot)
conn.wait_heartbeat()
print(f"Connected to system {conn.target_system}, component {conn.target_component}")

# Send GIMBAL_DEVICE_INFORMATION (sent once)
conn.mav.gimbal_device_information_send(
    int(time.time() * 1000) % (2**32),
    vendor_name=b"MockVendor".ljust(32, b'\0'),
    model_name=b"MockGimbal".ljust(32, b'\0'),
    custom_name=b"".ljust(32, b'\0'),
    firmware_version=100,  # 1.0.0
    hardware_version=1,
    uid=0,
    cap_flags=(
        mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT |
        mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL |
        mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS |
        mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
        mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
        mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW
    ),
    custom_cap_flags=0,
    roll_min=-3.14159, roll_max=3.14159,  # radians
    pitch_min=-3.14159, pitch_max=3.14159,
    yaw_min=-3.14159, yaw_max=3.14159
)

# Initial attitude quaternion (identity: no rotation)
current_q = [1.0, 0.0, 0.0, 0.0]
current_angular_velocity = [0.0, 0.0, 0.0]

# Variables to store received autopilot state and mission information
autopilot_state = None
current_mission_seq = None

last_heartbeat_time = time.time()
last_status_time = time.time()

print("Mock gimbal running. Sending heartbeats and attitude status, listening for autopilot and mission messages.")

while True:
    # Send heartbeat every 1 second
    if time.time() - last_heartbeat_time > 1:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GIMBAL,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,  # base_mode
            0,  # custom_mode
            mavutil.mavlink.MAV_STATE_ACTIVE
        )
        last_heartbeat_time = time.time()

    # Send attitude status every 0.1 seconds
    if time.time() - last_status_time > 0.1:
        conn.mav.gimbal_device_attitude_status_send(
            time_boot_ms=int(time.time() * 1000) % (2**32),
            target_system=0,  # Broadcast
            target_component=0,
            flags=0,
            q=current_q,
            angular_velocity_x=current_angular_velocity[0],
            angular_velocity_y=current_angular_velocity[1],
            angular_velocity_z=current_angular_velocity[2],
            failure_flags=0
        )
        last_status_time = time.time()

    # Receive messages
    msg = conn.recv_match(blocking=False)
    if msg is not None:
        msg_type = msg.get_type()
        
        if msg_type == 'GIMBAL_DEVICE_SET_ATTITUDE':
            print(f"Received GIMBAL_DEVICE_SET_ATTITUDE: flags={msg.flags}, q={msg.q}, angular_velocity=({msg.angular_velocity_x}, {msg.angular_velocity_y}, {msg.angular_velocity_z})")
            # Simulate setting the attitude
            current_q = list(msg.q)
            current_angular_velocity = [msg.angular_velocity_x, msg.angular_velocity_y, msg.angular_velocity_z]
            # Send ACK if required
            if msg.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ACK_REQUIRED:
                conn.mav.command_ack_send(
                    command=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                    result=mavutil.mavlink.MAV_RESULT_ACCEPTED
                )
        elif msg_type == 'AUTOPILOT_STATE_FOR_GIMBAL_DEVICE':

            pass
            #print(msg)
            # autopilot_state = {
            #     'time_boot_ms': msg.time_boot_us,
            #     'q_estimated': list(msg.q),
            #     'v_estimated': list(msg.v_estimated),
            #     'pos_estimated': list(msg.pos_estimated),
            #     'flags': msg.flags
            # }
            # print(f"Received AUTOPILOT_STATE_FOR_GIMBAL_DEVICE: time={msg.time_boot_ms}ms, "
            #       f"attitude_quaternion={msg.q_estimated}, "
            #       f"velocity={msg.v_estimated}, position={msg.pos_estimated}, flags={msg.flags}")
            # Example: Adjust gimbal based on autopilot state (simplified)
            # For now, just log it; in a real gimbal, you might use this for stabilization or tracking
        elif msg_type == 'MISSION_CURRENT':
            current_mission_seq = msg.seq
            print(f"Received MISSION_CURRENT: current mission sequence={msg.seq}")
            # Example: Use mission sequence to trigger specific gimbal actions
            # For now, just log it; in a real gimbal, you might adjust behavior based on mission item
        else:
            print(msg.id)
    time.sleep(0.001)  # Small sleep to avoid CPU overload