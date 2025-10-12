#!/usr/bin/env python3

import os, sys
import time

# force MAVLink 2.0
#os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

# ---------------------------------------------------------
def init_mavlink():
   # master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    master = mavutil.mavlink_connection("udpin:0.0.0.0:14540")
#    master = mavutil.mavlink_connection("udpin:0.0.0.0:14565", source_component=27, source_system=1)
   # master = mavutil.mavlink_connection("tcp:10.41.200.2:5790")
    print("Waiting for HB")
    master.wait_heartbeat()
    print("HB receiced")
    return master

# ---------------------------------------------------------
def send_manual_control_command(master, x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        int(x * 1000),
        int(y * 1000),
        int(z * 1000),
        int(r * 1000),
        0
    )

# ---------------------------------------------------------
def send_set_mode(master, base_mode, custom_mode, custom_sub_mode):

    master.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1,
        base_mode, custom_mode, custom_sub_mode, 0, 0, 0, 0
    )
    print(f"Mode sent! {custom_mode}  {custom_sub_mode}")
   # time.sleep(0.01)

# ---------------------------------------------------------
def set_mode(master, mode_str):

    try:
        (mode, custom_mode, custom_sub_mode) = master.mode_mapping()[mode_str]
        #master.set_mode(mode, custom_mode, custom_sub_mode)
        send_set_mode(master, mode, custom_mode, custom_sub_mode)

    except:
        mode = master.mode_mapping()[mode_str]
        #master.set_mode(mode, 0, 0)
        send_set_mode(master, mode, 0)

    cnt = 0
    while True:
        cnt += 1

        print(cnt)
        if cnt == 100:
            print(f"[-] (MAVLink) timeout while waiting for set_mode ack")
            sys.exit(-1)

        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=13)
        ack_msg = ack_msg.to_dict()

        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        if ack_msg["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"[+] (MAVLink) mode set: {mode_str}")
            break
        else:
            print(f"[-] (MAVLink) failed to set mode {mode_str}")
            sys.exit(-1)
    time.sleep(1)



# ---------------------------------------------------------
def arm(master):
    master.mav.command_long_send(
       # master.target_system,
       # master.target_component,
        1,
        1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()

    print("[+] (MAVLink) armed")
    time.sleep(1)

# ---------------------------------------------------------
def send_command_takeoff(master, alt_rel=10.0):
    
    print(f"[+] (MAVLink) get global position")

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg:
        lat = msg.lat*1e-7
        lon = msg.lon*1e-7
        alt = msg.alt*1e-3
        
    print(f"[+] (MAVLink) send takeoff command")
    master.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        lat,
        lon, #6.4541519,
        alt+alt_rel
    )
    time.sleep(1)


if __name__ == "__main__":
    master = init_mavlink()

    print( master.mode_mapping())
   # set_mode(master, 'RTL') # // Works

    set_mode(master, 'LOITER') # // Vanilla PX4 {'MANUAL': (81, 1, 0), 'STABILIZED': (81, 7, 0), 'ACRO': (65, 5, 0), 'RATTITUDE': (65, 8, 0), 'ALTCTL': (81, 2, 0), 'POSCTL': (81, 3, 0), 'LOITER': (29, 4, 3), 'MISSION': (29, 4, 4), 'RTL': (29, 4, 5), 'LAND': (29, 4, 6), 'RTGS': (29, 4, 7), 'FOLLOWME': (29, 4, 8), 'OFFBOARD': (29, 6, 0), 'TAKEOFF': (29, 4, 2)}

    # Send a dummy manual control command to keep the RC connected
    #send_manual_control_command(master, 0, 0, 0.8, 0)
    arm(master)
   # set_mode(master, mode_str="MANUAL") # also works for POSCTL, ALTCTL, ACRO
   # set_mode(master, mode_str="POSCTL") # also works for POSCTL, ALTCTL, ACRO
    time.sleep(3)
    #set_custom_mode(master, 30, "SWRM")


    send_command_takeoff(master, alt_rel=6.0)
    time.sleep(8)

   # set_mode(master, 'LAND') # // Works
   # time.sleep(6)
    set_mode(master, 'RTL') # // Works
    