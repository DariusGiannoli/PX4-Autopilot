"""Fire throw_sysid's hover pulses over MAVLink and print kf_b1's fit. Never arms, takes off or lands.
Usage: python3 hover_pulses.py --url udpin:0.0.0.0:14550 [--after-airborne 10] [--param THRSID_AMP=0.08 ...]
"""
import argparse
import re
import struct
import time

import numpy as np
from pymavlink import mavutil

from b1_watch import DEBUG_ARRAY_ID, NAMES, PATTERNS

SHELL_DEVICE = 10  # SERIAL_CONTROL_DEV_SHELL


class Px4Link:
    def __init__(self, url, baud):
        self.m = mavutil.mavlink_connection(url, baud=baud, source_system=247)
        t_end = time.monotonic() + 20
        while True:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if hb is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID \
                    and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
            if time.monotonic() > t_end:
                raise SystemExit(f"no autopilot heartbeat on {url}")
        self.armed, self.landed_state, self.b1 = False, 0, None
        self.shell_buf = ""
        for msg_id, hz in [(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5),
                           (mavutil.mavlink.MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, 10)]:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    def poll(self, timeout=0.05):
        t_end = time.monotonic() + timeout
        while True:
            msg = self.m.recv_match(blocking=True, timeout=max(0.0, t_end - time.monotonic()))
            if msg is None:
                return
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcComponent() == self.m.target_component \
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "EXTENDED_SYS_STATE":
                self.landed_state = msg.landed_state
            elif t == "DEBUG_FLOAT_ARRAY" and msg.array_id == DEBUG_ARRAY_ID:
                self.b1 = list(msg.data[:21])
            elif t == "SERIAL_CONTROL" and msg.count:
                self.shell_buf += "".join(chr(x) for x in msg.data[:msg.count])

    # -- shell (SERIAL_CONTROL)
    def shell(self, cmd, wait=1.5):
        self.shell_buf = ""
        data = (cmd + "\n").encode()
        while data:
            chunk, data = data[:70], data[70:]
            self.m.mav.serial_control_send(SHELL_DEVICE, mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                                           | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND, 0, 0, len(chunk),
                                           list(chunk) + [0] * (70 - len(chunk)))
        t_end = time.monotonic() + wait
        while time.monotonic() < t_end:
            self.poll(0.05)
            out = self.shell_buf.replace("\r", "")
            if out.count("> ") >= 1 and out.rstrip().endswith(">") and len(out.splitlines()) > 1:
                break
        out = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", self.shell_buf.replace("\r", ""))
        return "\n".join(ln for ln in out.splitlines() if ln.strip() and not ln.strip().endswith(">") and cmd not in ln)

    def close_shell(self):
        self.m.mav.serial_control_send(SHELL_DEVICE, 0, 0, 0, 0, [0] * 70)

    # -- parameters
    def get_param(self, name, timeout=3.0):
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is not None and msg.param_id.strip("\x00") == name:
                if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    return struct.unpack("<i", struct.pack("<f", msg.param_value))[0], msg.param_type
                return msg.param_value, msg.param_type
        return None, None

    def set_param(self, name, value):
        _, ptype = self.get_param(name)
        if ptype is None:
            raise SystemExit(f"parameter {name} not found on the vehicle")
        if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            raw = struct.unpack("<f", struct.pack("<i", int(float(value))))[0]
        else:
            raw = float(value)
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), raw, ptype)
        got, _ = self.get_param(name)
        return got


def own_gains(b1):
    g = np.array(b1[:16]).reshape(4, 4)
    return np.array([PATTERNS[o] @ g[o] for o in range(4)])


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--url", required=True, help="MAVLink connection, e.g. udpin:0.0.0.0:14550 or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--param", action="append", default=[], metavar="NAME=VALUE", help="set before the pulses (repeatable)")
    p.add_argument("--timeout", type=float, default=40.0, help="stop monitoring after this [s]")
    p.add_argument("--after-airborne", type=float, metavar="S",
                   help="start the pulses by itself once the vehicle has been in the air for S seconds (no Enter)")
    a = p.parse_args()

    v = Px4Link(a.url, a.baud)
    print(f"connected: system {v.m.target_system}")
    for kv in a.param:
        name, val = kv.split("=", 1)
        print(f"  {name} = {v.set_param(name, val)}")

    for mod in ("kf_b1", "throw_sysid"):
        if "not running" in v.shell(f"{mod} status"):
            print(f"starting {mod}")
            v.shell(f"{mod} start")
    status = v.shell("throw_sysid status")
    print("\n".join("  " + ln.split("] ", 1)[-1] for ln in status.splitlines() if "throw_sysid]" in ln))

    v.poll(1.0)
    if a.after_airborne is not None:
        print(f"waiting until the vehicle has been in the air for {a.after_airborne:.0f} s (Ctrl-C to quit) ...")
        since = None
        while since is None or time.monotonic() - since < a.after_airborne:
            v.poll(0.2)
            in_air = v.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR
            since = (since or time.monotonic()) if in_air else None
    else:
        if v.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("note: the vehicle is on the ground; take off and hover before pressing Enter")
        input("\nPress Enter to start the pulses (Ctrl-C to quit) ")

    out = v.shell("throw_sysid arm")
    print("\n".join("  " + re.sub(r"^.*?(INFO|WARN|ERROR)\s+\[\w+\]\s*", "", ln) for ln in out.splitlines()) or "  armed")
    phase, seen_active, t_end = None, False, time.monotonic() + a.timeout
    try:
        while time.monotonic() < t_end:
            m = re.search(r"phase: (\w+)[^,]*, last end: ([^,]+)", v.shell("throw_sysid status", wait=0.6))
            if m and m.group(1) != phase:
                phase = m.group(1)
                print(f"  {time.strftime('%H:%M:%S')} phase {phase}" + (f" ({m.group(2)})" if phase == "DONE" else ""))
                seen_active |= phase == "EXCITE"
            if phase == "DONE" or (seen_active and phase == "IDLE"):
                break
            v.poll(0.3)
    except KeyboardInterrupt:
        print("\ncancel")
        v.shell("throw_sysid cancel")

    v.poll(1.0)
    print("\nkf_b1 now (rows a_z, alpha_x, alpha_y, alpha_z; cols FL, RL, RR, FR), then the last throw fit:")
    print("\n".join(ln.split("] ", 1)[-1] for ln in v.shell("kf_b1 status").splitlines()
                           if "kf_b1]" in ln and ("throw" in ln or "started from" in ln or re.search(r"^\s*-?\d", ln.split("] ", 1)[-1]))))
    if v.b1 is not None:
        g = own_gains(v.b1)
        print("own gains: " + ", ".join(f"{NAMES[o].split()[0]} {g[o]:.2f}" for o in range(4))
              + f" (started from {'throw' if v.b1[17] > 0.5 else 'prior'})")
    v.close_shell()


if __name__ == "__main__":
    main()
