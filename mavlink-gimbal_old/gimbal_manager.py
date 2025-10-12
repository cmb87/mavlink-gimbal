#!/usr/bin/env python3
"""
Minimal MAVLink Gimbal Manager + Mock Gimbal

- Listens on UDP for COMMAND_LONG (MAV_CMD_DO_MOUNT_CONTROL, #205)
- Simulates a gimbal with limits and slew rates
- Publishes HEARTBEAT (as a gimbal component) and ATTITUDE as status
- Includes a tiny client to send commands for testing

Dependencies:
  pip install pymavlink
"""

import math
import time
import threading
import argparse
from dataclasses import dataclass

from pymavlink import mavutil


# ----------------------------
# Mock Gimbal Model
# ----------------------------
@dataclass
class GimbalLimits:
    pitch_min_deg: float = -90.0
    pitch_max_deg: float =  30.0
    yaw_min_deg:   float = -180.0
    yaw_max_deg:   float =  180.0
    roll_min_deg:  float = -45.0
    roll_max_deg:  float =  45.0

@dataclass
class GimbalSlewRates:
    pitch_deg_s: float = 60.0
    yaw_deg_s:   float = 90.0
    roll_deg_s:  float = 60.0

class MockGimbal:
    """
    A very simple 3-axis gimbal simulator:
    - clamps targets to limits
    - moves toward target at configured slew rate
    - stores current angles (deg) and angular rates (rad/s for MAVLink ATTITUDE)
    """
    def __init__(self, limits=None, slew=None):
        self.limits = limits or GimbalLimits()
        self.slew = slew or GimbalSlewRates()

        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0
        self.roll_deg  = 0.0

        self.tgt_pitch_deg = 0.0
        self.tgt_yaw_deg   = 0.0
        self.tgt_roll_deg  = 0.0

        self.pitch_rate_rads = 0.0
        self.yaw_rate_rads   = 0.0
        self.roll_rate_rads  = 0.0

        self._lock = threading.Lock()

    def set_target(self, pitch_deg: float, yaw_deg: float, roll_deg: float):
        with self._lock:
            self.tgt_pitch_deg = max(self.limits.pitch_min_deg, min(self.limits.pitch_max_deg, pitch_deg))
            self.tgt_yaw_deg   = max(self.limits.yaw_min_deg,   min(self.limits.yaw_max_deg,   yaw_deg))
            self.tgt_roll_deg  = max(self.limits.roll_min_deg,  min(self.limits.roll_max_deg,  roll_deg))

    def _slew_axis(self, current, target, rate_deg_s, dt):
        if dt <= 0:
            return current, 0.0
        delta = target - current
        max_step = rate_deg_s * dt
        step = max(-max_step, min(max_step, delta))
        new_val = current + step
        rate_rads = math.radians(step) / dt if dt > 0 else 0.0
        return new_val, rate_rads

    def update(self, dt: float):
        with self._lock:
            self.pitch_deg, self.pitch_rate_rads = self._slew_axis(self.pitch_deg, self.tgt_pitch_deg, self.slew.pitch_deg_s, dt)
            self.yaw_deg,   self.yaw_rate_rads   = self._slew_axis(self.yaw_deg,   self.tgt_yaw_deg,   self.slew.yaw_deg_s,   dt)
            self.roll_deg,  self.roll_rate_rads  = self._slew_axis(self.roll_deg,  self.tgt_roll_deg,  self.slew.roll_deg_s,  dt)

    def get_state(self):
        with self._lock:
            return {
                "pitch_deg": self.pitch_deg,
                "yaw_deg":   self.yaw_deg,
                "roll_deg":  self.roll_deg,
                "pitch_rate_rads": self.pitch_rate_rads,
                "yaw_rate_rads":   self.yaw_rate_rads,
                "roll_rate_rads":  self.roll_rate_rads,
            }


# ----------------------------
# MAVLink Gimbal Manager
# ----------------------------
class GimbalManager:
    """
    Minimal, pragmatic "manager":
    - Identifies as component MAV_COMP_ID_GIMBAL within a MAVLink system
    - Accepts MAV_CMD_DO_MOUNT_CONTROL: params (pitch, roll, yaw) in degrees
    - ACKs every command
    - Periodically publishes HEARTBEAT and ATTITUDE as status
    """

    def __init__(self, bind_ip: str, bind_port: int, system_id: int = 42, component_id: int = mavutil.mavlink.MAV_COMP_ID_GIMBAL):
        # Listen on UDP (incoming)
        self.mav = mavutil.mavlink_connection(f"udpin:{bind_ip}:{bind_port}",
                                              source_system=system_id,
                                              source_component=component_id,
                                              dialect="common")
        self.gimbal = MockGimbal()
        self._running = False
        self._last_att_ms = 0
        self._status_hz = 20.0      # ATTITUDE publish rate
        self._hb_hz = 1.0           # HEARTBEAT rate

        # track a "time_boot_ms"
        self._boot_time = time.time()

    def _time_boot_ms(self):
        return int((time.time() - self._boot_time) * 1000.0)

    # --------- send helpers ----------
    def send_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GIMBAL,              # type
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,        # autopilot (not an autopilot)
            0,                                            # base_mode
            0,                                            # custom_mode
            mavutil.mavlink.MAV_STATE_ACTIVE              # system_status
        )

    def send_attitude(self):
        s = self.gimbal.get_state()
        # MAVLink ATTITUDE uses radians; roll=x, pitch=y, yaw=z (right-hand)
        roll  = math.radians(s["roll_deg"])
        pitch = math.radians(s["pitch_deg"])
        yaw   = math.radians(s["yaw_deg"])

        self.mav.mav.attitude_send(
            self._time_boot_ms(),
            roll, pitch, yaw,
            s["roll_rate_rads"], s["pitch_rate_rads"], s["yaw_rate_rads"]
        )

    def send_statustext(self, txt: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
        # Trim to MAVLink STATUSTEXT v2 maximum (50 chars for text, pymavlink handles v2 chunking)
        self.mav.mav.statustext_send(severity, txt.encode("utf-8"))

    def send_command_ack(self, command, result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        self.mav.mav.command_ack_send(command, result)

    # --------- command handling ----------
    def _handle_command_long(self, msg):
        cmd = msg.command
        if cmd == mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL:
            # params: [1]=pitch deg, [2]=roll deg, [3]=yaw deg, rest unused here
            pitch = msg.param1
            roll  = msg.param2
            yaw   = msg.param3

            # Some ground stations send NANs when not used; treat NAN as "keep current"
            s = self.gimbal.get_state()
            if math.isnan(pitch): pitch = s["pitch_deg"]
            if math.isnan(roll):  roll  = s["roll_deg"]
            if math.isnan(yaw):   yaw   = s["yaw_deg"]

            self.gimbal.set_target(pitch, yaw, roll)
            self.send_command_ack(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            self.send_statustext(f"Set mount: pitch={pitch:.1f} yaw={yaw:.1f} roll={roll:.1f}")
        else:
            # Unsupported command
            self.send_command_ack(cmd, mavutil.mavlink.MAV_RESULT_UNSUPPORTED)
            self.send_statustext(f"Unsupported command {cmd}", mavutil.mavlink.MAV_SEVERITY_WARNING)

    # --------- main loops ----------
    def _rx_loop(self):
        """Receive and process incoming MAVLink messages."""
        while self._running:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.002)
                continue

            mtype = msg.get_type()

            if mtype == "BAD_DATA":
                continue

            if mtype == "COMMAND_LONG":
                self._handle_command_long(msg)

            # You could extend here to handle e.g. PARAM_* for limits/slew, REQUEST_MESSAGE, etc.

    def _status_loop(self):
        """Publish HEARTBEAT and ATTITUDE while updating the simulation."""
        last_hb = 0.0
        last = time.time()
        att_interval = 1.0 / self._status_hz
        hb_interval = 1.0 / self._hb_hz
        att_accum = 0.0

        while self._running:
            now = time.time()
            dt = now - last
            last = now

            # advance the gimbal simulation
            self.gimbal.update(dt)

            # ATTITUDE at ~20 Hz
            att_accum += dt
            if att_accum >= att_interval:
                self.send_attitude()
                att_accum = 0.0

            # HEARTBEAT at ~1 Hz
            if (now - last_hb) >= hb_interval:
                self.send_heartbeat()
                last_hb = now

            time.sleep(0.002)

    def run(self):
        self._running = True
        self.send_statustext("Gimbal manager starting…")

        rx_t = threading.Thread(target=self._rx_loop, daemon=True)
        st_t = threading.Thread(target=self._status_loop, daemon=True)
        rx_t.start()
        st_t.start()

        try:
            while True:
                time.sleep(0.25)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self.send_statustext("Gimbal manager stopping…")
            time.sleep(0.1)  # flush
            try:
                self.mav.close()
            except Exception:
                pass


# ----------------------------
# Tiny client (for testing)
# ----------------------------
def send_mount_control(target_ip: str, target_port: int,
                       system_id: int, component_id: int,
                       pitch_deg: float, yaw_deg: float, roll_deg: float,
                       wait_ack: bool = True):
    """
    Send MAV_CMD_DO_MOUNT_CONTROL to a target UDP endpoint.
    """
    mav = mavutil.mavlink_connection(
        f"udpout:{target_ip}:{target_port}",
        source_system=system_id,
        source_component=component_id,
        dialect="common"
    )

    # Send a heartbeat first so the other side knows our IDs
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )
    time.sleep(0.05)

    # Command: MAV_CMD_DO_MOUNT_CONTROL (205)
    mav.mav.command_long_send(
        target_system=0,   # 0=broadcast (manager doesn’t filter by target here)
        target_component=0,
        command=mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        confirmation=0,
        param1=float(pitch_deg),  # pitch (deg)
        param2=float(roll_deg),   # roll  (deg)
        param3=float(yaw_deg),    # yaw   (deg)
        param4=0, param5=0, param6=0, param7=0
    )

    print(f"Sent mount control: pitch={pitch_deg} roll={roll_deg} yaw={yaw_deg}")

    if wait_ack:
        t0 = time.time()
        while time.time() - t0 < 2.0:
            msg = mav.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=False)
            if msg:
                print(f"<- {msg}")
                if msg.get_type() == "COMMAND_ACK":
                    break
            time.sleep(0.01)
    mav.close()


# ----------------------------
# CLI
# ----------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Minimal MAVLink Gimbal Manager + Mock Gimbal")
    sub = p.add_subparsers(dest="mode", required=True)

    m = sub.add_parser("manager", help="Run the gimbal manager (server)")
    m.add_argument("--listen", default="0.0.0.0:14550", help="Bind IP:port to listen for MAVLink (default 0.0.0.0:14550)")
    m.add_argument("--sysid", type=int, default=42, help="MAVLink system ID for the manager (default 42)")
    m.add_argument("--compid", type=int, default=mavutil.mavlink.MAV_COMP_ID_GIMBAL, help="MAVLink component ID (default gimbal)")

    c = sub.add_parser("client", help="Run a tiny client that sends a mount control command")
    c.add_argument("--target", default="127.0.0.1:14550", help="Target IP:port of the manager (default 127.0.0.1:14550)")
    c.add_argument("--sysid", type=int, default=13, help="Client System ID (default 13)")
    c.add_argument("--compid", type=int, default=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER, help="Client Component ID (default MP)")
    c.add_argument("--pitch", type=float, default=0.0, help="Pitch deg (down is negative in many UIs; this script uses math-positive up)")
    c.add_argument("--yaw",   type=float, default=0.0, help="Yaw deg")
    c.add_argument("--roll",  type=float, default=0.0, help="Roll deg")
    c.add_argument("--noack", action="store_true", help="Do not wait for COMMAND_ACK")

    return p.parse_args()

def main():
    args = parse_args()
    if args.mode == "manager":
        ip, port = args.listen.split(":")
        gm = GimbalManager(ip, int(port), system_id=args.sysid, component_id=args.compid)
        gm.run()

    elif args.mode == "client":
        ip, port = args.target.split(":")
        send_mount_control(
            target_ip=ip, target_port=int(port),
            system_id=args.sysid, component_id=args.compid,
            pitch_deg=args.pitch, yaw_deg=args.yaw, roll_deg=args.roll,
            wait_ack=not args.noack
        )

if __name__ == "__main__":
    main()