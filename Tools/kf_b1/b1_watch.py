"""Live view of the kf_b1 estimate: B1, motor-pattern gains and error against a reference.
Usage: python3 b1_watch.py [--url udpin:0.0.0.0:14550 | --instance 1] [--ref testdrone|x500lin|x500|sih] [--ref-volt V]
"""
import argparse
import os
import re
import subprocess
import time

import numpy as np

PX4_BIN = os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/bin")
# rows a_z, alpha_x, alpha_y, alpha_z (FLU); columns FL, RL, RR, FR
PATTERNS = np.array([[1, 1, 1, 1], [1, 1, -1, -1], [-1, 1, 1, -1], [1, -1, 1, -1]], float) / 4
TESTDRONE_VREF = 16.8


def testdrone_b1(volt=TESTDRONE_VREF):
    """Test drone B1 per unit motor command from the drone model (T_i/m = 12.68 u_i + 0.183, u_i = cmd V/16.8)."""
    k = 12.68 * volt / TESTDRONE_VREF
    return k * np.array([[1, 1, 1, 1], [31.65, 31.65, -31.65, -31.65], [-29.70, 29.70, 29.70, -29.70],
                         [4.03, -4.03, 4.03, -4.03]])


REFS = {
    # test drone, at TESTDRONE_VREF (--ref-volt rescales it)
    "testdrone": testdrone_b1(),
    "sih": np.array([[5, 5, 5, 5], [40, 40, -40, -40], [-40, 40, 40, -40], [10 / 3, -10 / 3, 10 / 3, -10 / 3]]),
    # x500 with THR_MDL_FAC = 0, slope at hover
    "x500": np.array([[5.4] * 4, [90, 90, -90, -90], [-90, 90, 90, -90], [4.5, -4.5, 4.5, -4.5]]),
    # x500 with THR_MDL_FAC = 0.67, per unit of relative thrust
    "x500lin": np.array([[4.15] * 4, [68.7, 68.7, -68.7, -68.7], [-68.7, 68.7, 68.7, -68.7], [3.42, -3.42, 3.42, -3.42]]),
}
NAMES = ["thrust (a_z)", "roll (alpha_x)", "pitch (alpha_y)", "yaw (alpha_z)"]
DEBUG_ARRAY_ID = 0xB1  # kf_b1's DEBUG_FLOAT_ARRAY: data[0..15] B1, 16 valid, 17 init_source, 18 steps, 19 armed, 20 landed


def physical_b1(mass, arm_x, arm_y, t_max, ixx, iyy, izz, k_m):
    """B1 per unit command of a quad X with thrust linear in the command (T = t_max * u)."""
    x = np.array([arm_x, -arm_x, -arm_x, arm_x])          # FL, RL, RR, FR (FLU: x forward, y left)
    y = np.array([arm_y, arm_y, -arm_y, -arm_y])
    spin = np.array([1, -1, 1, -1])                         # yaw reaction: FL, RR cw -> +, RL, FR ccw -> -
    return np.vstack([np.full(4, t_max / mass), t_max * y / ixx, -t_max * x / iyy, spin * k_m * t_max / izz])


def read_status(instance):
    """SITL only: parse 'kf_b1 status' from the local PX4 instance."""
    cmd = [os.path.join(PX4_BIN, "px4-kf_b1")] + (["--instance", str(instance)] if instance else []) + ["status"]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
    head = re.search(r"armed: (\d), landed: (\d), motor command: (\d), KF steps since arming: (\d+)", out)
    rows = re.findall(r"\[kf_b1\]\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*$", out, re.M)
    if not head or len(rows) < 4:
        return None
    src = "throw" if "started from: throw" in out else "prior"
    return dict(armed=int(head[1]), landed=int(head[2]), steps=int(head[4]), G=np.array(rows[-4:], float), init=src)


class MavlinkSource:
    """SITL or hardware: kf_b1's DEBUG_FLOAT_ARRAY over any MAVLink connection."""

    def __init__(self, url, baud, rate_hz):
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(url, baud=baud, source_system=246)
        hb = self.m.wait_heartbeat(timeout=20)
        if hb is None:
            raise SystemExit(f"no heartbeat on {url}")
        # request the stream on this link
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                     mavutil.mavlink.MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, 1e6 / rate_hz, 0, 0, 0, 0, 0)
        self.last = None

    def read(self, timeout):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            msg = self.m.recv_match(type="DEBUG_FLOAT_ARRAY", blocking=True, timeout=0.1)
            if msg is not None and msg.array_id == DEBUG_ARRAY_ID:
                d = msg.data
                self.last = dict(G=np.array(d[:16], float).reshape(4, 4), init="throw" if d[17] > 0.5 else "prior",
                                 steps=int(d[18]), armed=int(d[19] > 0.5), landed=int(d[20] > 0.5))
        return self.last


def show(s, ref):
    state = "in air" if s["armed"] and not s["landed"] else ("armed, on ground" if s["armed"] else "disarmed")
    print(f"\n[{time.strftime('%H:%M:%S')}] {state}, KF steps since arming {s['steps']}, started from {s['init']}")
    print("  B1 (rows a_z, alpha_x, alpha_y, alpha_z; cols FL, RL, RR, FR):")
    for r in s["G"]:
        print("   " + " ".join(f"{v:9.2f}" for v in r))
    gains = np.array([PATTERNS[o] @ s["G"][o] for o in range(4)])
    for o in range(4):
        line = f"  {NAMES[o]:16s} {gains[o]:9.2f}"
        if ref is not None:
            rg = PATTERNS[o] @ ref[o]
            line += f"   ref {rg:7.2f}   error {100 * (gains[o] - rg) / abs(rg):+6.1f}%"
        print(line)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--url", help="MAVLink connection (hardware or SITL), e.g. udpin:0.0.0.0:14550, /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--rate", type=float, default=10.0, help="requested DEBUG_FLOAT_ARRAY rate with --url [Hz]")
    p.add_argument("--instance", type=int, default=0, help="without --url: local PX4 SITL instance (Gazebo = 1)")
    p.add_argument("--ref", choices=list(REFS), help="reference B1 to compare against (simulation models)")
    p.add_argument("--ref-volt", type=float, help="battery voltage for --ref testdrone [V] (default 16.8)")
    p.add_argument("--ref-phys", type=float, nargs=8, metavar=("MASS", "ARM_X", "ARM_Y", "T_MAX", "IXX", "IYY", "IZZ", "K_M"),
                   help="reference from physical data [kg, m, m, N per motor, kg m^2 x3, torque/thrust]")
    p.add_argument("--ref-file", help="reference from a text file: 4x4 B1 (rows a_z, alpha_x, alpha_y, alpha_z; cols FL RL RR FR)")
    p.add_argument("--interval", type=float, default=1.0, help="print interval [s]")
    p.add_argument("--once", action="store_true", help="print once and exit")
    a = p.parse_args()
    ref = REFS.get(a.ref)
    if a.ref == "testdrone" and a.ref_volt:
        ref = testdrone_b1(a.ref_volt)
    if a.ref_phys:
        ref = physical_b1(*a.ref_phys)
    elif a.ref_file:
        ref = np.loadtxt(a.ref_file).reshape(4, 4)
    src = MavlinkSource(a.url, a.baud, a.rate) if a.url else None
    while True:
        s = src.read(a.interval) if src else read_status(a.instance)
        if s is None:
            print("no kf_b1 data (" + ("no DEBUG_FLOAT_ARRAY from kf_b1 yet" if src else
                                       "PX4 not running, wrong --instance, or kf_b1 not started") + ")")
        else:
            show(s, ref)
        if a.once:
            break
        if not src:
            time.sleep(a.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
