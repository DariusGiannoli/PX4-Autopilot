"""SITL stand-in for an external controller: trajectory file -> cascaded PID -> body rates + thrust over MAVLink.
Usage: python3 sitl_controller.py --traj ../trajectories/hoverTraj.py --arm   (Enter = trajectory_begin)
"""
import argparse
import importlib.util
import math
import sys
import threading
import time

import numpy as np
from pymavlink import mavutil

G = 9.81
MAIN_AUTO, MAIN_OFFBOARD, SUB_AUTO_LAND = 4, 6, 6
TYPEMASK_ATTITUDE_IGNORE = 128


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_traj(path):
    """Instance of the trajectory class (the one with calculate_trajectory) defined in the file."""
    spec = importlib.util.spec_from_file_location("traj", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    cls = [c for c in vars(mod).values() if isinstance(c, type) and c.__module__ == "traj" and hasattr(c, "calculate_trajectory")]
    return cls[0]()


def enu_to_ned(ref, origin):
    """Trajectory output (x east, y north, z up, yaw from x) -> PX4 local NED position, velocity, acceleration, yaw."""
    x, y, z, vx, vy, vz, ax, ay, az, yaw = ref
    return origin + np.array([y, x, -z]), np.array([vy, vx, -vz]), np.array([ay, ax, -az]), wrap(math.pi / 2 - yaw)


def quat_to_rot(q):
    """Quaternion w, x, y, z (FRD body -> NED) -> rotation matrix."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Link:
    """MAVLink connection to PX4: telemetry, mode and arming commands, body-rate setpoints."""

    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=245)
        while True:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=20)
            if hb is None:
                raise SystemExit(f"no heartbeat on {url}")
            if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        self.pos = self.vel = self.q = None
        self.t_pos, self.armed, self.main_mode, self.landed_state = 0.0, False, 0, 0
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50),
                           (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100),
                           (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5)):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def command(self, cmd, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def poll(self):
        while (msg := self.m.recv_match(blocking=False)) is not None:
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.pos, self.vel, self.t_pos = np.array([msg.x, msg.y, msg.z]), np.array([msg.vx, msg.vy, msg.vz]), time.monotonic()
            elif t == "ATTITUDE_QUATERNION":
                self.q = np.array([msg.q1, msg.q2, msg.q3, msg.q4])
            elif t == "HEARTBEAT" and msg.get_srcComponent() == self.m.target_component and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.main_mode = (msg.custom_mode >> 16) & 0xFF
            elif t == "EXTENDED_SYS_STATE":
                self.landed_state = msg.landed_state
            elif t == "STATUSTEXT":
                print(f"  [PX4] {msg.text}")

    def get_param(self, name):
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
        t_end = time.monotonic() + 3
        while time.monotonic() < t_end:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is not None and msg.param_id.strip("\x00") == name:
                return msg.param_value
        return None

    def set_mode(self, main, sub=0):
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def send_rates(self, rates, thrust):
        """Body rates (FRD, rad/s) and normalized collective thrust [0, 1]."""
        self.m.mav.set_attitude_target_send(0, self.m.target_system, self.m.target_component, TYPEMASK_ATTITUDE_IGNORE,
                                            [1.0, 0.0, 0.0, 0.0], *(float(r) for r in rates), float(thrust))


class Controller:
    """Position P -> velocity PI -> attitude (thrust direction + yaw) -> attitude P -> body rates; thrust from the hover thrust."""

    def __init__(self, a, hover_thrust):
        self.a, self.hover = a, hover_thrust
        self.kp_pos = np.array([a.kp_pos_xy, a.kp_pos_xy, a.kp_pos_z])
        self.kp_vel = np.array([a.kp_vel_xy, a.kp_vel_xy, a.kp_vel_z])
        self.ki_vel = np.array([a.ki_vel_xy, a.ki_vel_xy, a.ki_vel_z])
        self.k_att = np.array([a.kp_att_rp, a.kp_att_rp, a.kp_att_yaw])
        self.rate_max = np.array([a.rate_max_rp, a.rate_max_rp, a.rate_max_yaw])
        self.i = np.zeros(3)

    def step(self, dt, pos, vel, q, ref, yaw_rate_ff, airborne, t):
        a = self.a
        p_r, v_r, a_r, yaw_r = ref
        v_cmd = v_r + self.kp_pos * (p_r - pos)
        n_xy = np.linalg.norm(v_cmd[:2])
        if n_xy > a.v_max_xy:
            v_cmd[:2] *= a.v_max_xy / n_xy
        v_cmd[2] = np.clip(v_cmd[2], -a.v_max_z, a.v_max_z)
        e_v = v_cmd - vel
        acc = a_r + self.kp_vel * e_v + self.ki_vel * self.i
        f = np.array([0.0, 0.0, G]) - acc                 # specific force along body z (down), NED
        f[2] = max(f[2], 0.3 * G)
        tan_tilt, f_xy = math.tan(math.radians(a.tilt_max)), np.linalg.norm(f[:2])
        tilt_limited = f_xy > f[2] * tan_tilt
        if tilt_limited:
            f[:2] *= f[2] * tan_tilt / f_xy
        thrust = self.hover * np.linalg.norm(f) / G
        thrust_limited = not (a.thrust_min < thrust < a.thrust_max)
        thrust = float(np.clip(thrust, a.thrust_min, a.thrust_max))
        if t < a.spinup_s:                                 # thrust ramp after arming
            thrust = min(thrust, a.thrust_min + (self.hover - a.thrust_min) * t / a.spinup_s * 1.2)
        if airborne and not (tilt_limited or thrust_limited):
            self.i = np.clip(self.i + e_v * dt, -a.i_max, a.i_max)
        # desired attitude: body z along f, heading yaw_r
        z_b = f / np.linalg.norm(f)
        y_b = np.cross(z_b, [math.cos(yaw_r), math.sin(yaw_r), 0.0])
        y_b /= np.linalg.norm(y_b)
        R_d = np.column_stack([np.cross(y_b, z_b), y_b, z_b])
        # attitude error (body frame) -> body-rate command, plus the yaw-rate feedforward
        R = quat_to_rot(q)
        E = 0.5 * (R_d.T @ R - R.T @ R_d)
        e_R = np.array([E[2, 1], E[0, 2], E[1, 0]])
        rates = -self.k_att * e_R + R.T @ np.array([0.0, 0.0, yaw_rate_ff])
        return np.clip(rates, -self.rate_max, self.rate_max), thrust


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--traj", required=True, help="trajectory file (e.g. ../trajectories/hoverTraj.py)")
    p.add_argument("--url", default="udpin:0.0.0.0:14541", help="MAVLink connection (Gazebo instance 1: 14541)")
    p.add_argument("--arm", action="store_true", help="arm over MAVLink")
    p.add_argument("--hover_thrust", type=float, default=None, help="normalized hover thrust (default: MPC_THR_HOVER)")
    p.add_argument("--rate", type=float, default=100.0, help="control loop rate [Hz]")
    p.add_argument("--fence", type=float, default=2.5, help="land beyond this distance from the start point [m]")
    p.add_argument("--max_time", type=float, default=300.0, help="land after this flight time [s]")
    for k, d in dict(kp_pos_xy=1.0, kp_pos_z=1.2, kp_vel_xy=2.0, ki_vel_xy=0.4, kp_vel_z=3.0, ki_vel_z=1.0, i_max=2.0,
                     v_max_xy=3.0, v_max_z=1.5, tilt_max=35.0, thrust_min=0.08, thrust_max=0.9, spinup_s=1.0,
                     kp_att_rp=6.5, kp_att_yaw=2.8, rate_max_rp=3.5, rate_max_yaw=2.0).items():
        p.add_argument(f"--{k}", type=float, default=d)
    a = p.parse_args()

    traj = load_traj(a.traj)
    print(f"trajectory {type(traj).__name__}: {traj.total_time:.1f} s")
    v = Link(a.url)
    t_wait = time.monotonic() + 10
    while (v.pos is None or v.q is None) and time.monotonic() < t_wait:
        v.poll()
        time.sleep(0.02)
    if v.pos is None or v.q is None:
        raise SystemExit("no LOCAL_POSITION_NED / ATTITUDE_QUATERNION")
    if v.armed:
        raise SystemExit("vehicle already armed: start on the ground, disarmed")
    hover = a.hover_thrust if a.hover_thrust is not None else v.get_param("MPC_THR_HOVER")
    print(f"hover thrust {hover:.3f}")
    ctrl, origin = Controller(a, hover), v.pos.copy()
    cross = threading.Event()
    threading.Thread(target=lambda: (sys.stdin.readline(), cross.set()), daemon=True).start()

    dt, t0, phase, t_end = 1.0 / a.rate, None, "hold", None
    t_start = t_next = time.monotonic()
    yaw_prev, t_prev = None, None
    try:
        while True:
            v.poll()
            now = time.monotonic()
            if t0 is None:                                 # stream setpoints, OFFBOARD, arm
                if now - t_start > 1.0 and v.main_mode != MAIN_OFFBOARD:
                    v.set_mode(MAIN_OFFBOARD)
                if v.main_mode == MAIN_OFFBOARD and not v.armed and a.arm:
                    v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                if v.main_mode == MAIN_OFFBOARD and v.armed:
                    t0 = now
                    print("OFFBOARD and armed: flying to the trajectory's start point; Enter = trajectory_begin")
                elif now - t_start > 30:
                    raise RuntimeError("not OFFBOARD + armed within 30 s")
                v.send_rates((0.0, 0.0, 0.0), 0.0)
            else:
                t = now - t0
                if now - v.t_pos > 0.5:
                    raise RuntimeError("position telemetry stale")
                if np.linalg.norm(v.pos - origin) > a.fence or t > a.max_time:
                    raise RuntimeError(f"fence or time limit: position {np.round(v.pos - origin, 2)} m, t {t:.0f} s")
                if v.main_mode != MAIN_OFFBOARD:
                    print("left OFFBOARD: stopped sending")
                    return
                if phase == "hold" and cross.is_set():
                    traj.trajectory_begin(t)
                    phase = "traj"
                    print("trajectory_begin")
                ref = enu_to_ned(traj.calculate_trajectory(t), origin)
                if phase == "traj" and not traj.traj_enable:
                    phase, t_end = "end", t
                    print("trajectory done")
                if phase == "end" and t - t_end > 2.0:
                    break
                yaw_rate = 0.0 if yaw_prev is None or t == t_prev else wrap(ref[3] - yaw_prev) / (t - t_prev)
                airborne = v.landed_state == 2 or (origin[2] - v.pos[2]) > 0.15
                rates, thrust = ctrl.step(dt if t_prev is None else t - t_prev, v.pos, v.vel, v.q, ref, yaw_rate, airborne, t)
                yaw_prev, t_prev = ref[3], t
                v.send_rates(rates, thrust)
            t_next += dt
            time.sleep(max(0.0, t_next - time.monotonic()))
    except (Exception, KeyboardInterrupt) as e:
        print(f"ABORT: {type(e).__name__}: {e}")
    if v.armed:
        print("landing (AUTO.LAND)")
        for _ in range(20):
            v.set_mode(MAIN_AUTO, SUB_AUTO_LAND)
            time.sleep(0.05)
    t_wait = time.monotonic() + 60
    while time.monotonic() < t_wait and v.armed:
        v.poll()
        time.sleep(0.1)
    print("disarmed" if not v.armed else "still armed after 60 s")


if __name__ == "__main__":
    main()
