import math
import numpy as np

# ---------------------------------------------------------------- parameters (z up, metres, seconds, radians)
X0, Y0 = 0.0, 0.0      # square centre; the trajectory starts and ends here at Z_LOW
SIDE = 1.0             # side length of the square
Z_LOW = 0.5            # height of the start point and of corners 2 and 4
Z_HIGH = 0.8           # height of corners 1 and 3 (= Z_LOW for a flat square)
LAPS = 2               # number of laps
V_MAX = 0.5            # peak speed of each corner-to-corner move
A_MAX = 1.0            # peak acceleration of each move
HOLD_S = 1.0           # hover time at each corner
MIN_MOVE_S = 0.5       # shortest move
YAW = 0.0              # constant heading


def _min_jerk(s):
    """Minimum-jerk blend 0 -> 1 and its first two derivatives with respect to s in [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    return (10 * s**3 - 15 * s**4 + 6 * s**5,
            30 * s**2 - 60 * s**3 + 30 * s**4,
            60 * s - 180 * s**2 + 120 * s**3)


def _move_time(dist, v_max, a_max):
    """Rest-to-rest min-jerk duration within v_max and a_max (peak v = 1.875 d/T, peak a = 5.774 d/T^2)."""
    if dist <= 0:
        return 0.0
    return max(1.875 * dist / v_max, math.sqrt(5.774 * dist / a_max))


class SquareTraj():
    def __init__(self, x0=X0, y0=Y0, side=SIDE, z_low=Z_LOW, z_high=Z_HIGH, laps=LAPS,
                 v_max=V_MAX, a_max=A_MAX, hold_s=HOLD_S, yaw=YAW):
        """
        Square laps with alternating corner heights and constant yaw, stopping at every corner.

        :param x0, y0: Square centre (start and end point, at z_low)
        :param side: Side length
        :param z_low, z_high: Corner heights (corners alternate high / low)
        :param laps: Number of laps
        :param v_max, a_max: Peak speed and acceleration of each move
        :param hold_s: Hover time at each corner
        :param yaw: Constant heading (rad)
        """
        self.x0 = x0
        self.y0 = y0
        self.z0 = z_low
        self.yaw = yaw
        self.start_time = 0
        self.traj_enable = False

        h = side / 2
        corners = [(h, h), (-h, h), (-h, -h), (h, -h)]
        waypoints = []
        for _ in range(laps):
            for i, (dx, dy) in enumerate(corners):
                waypoints.append((x0 + dx, y0 + dy, z_high if i % 2 == 0 else z_low))
        waypoints.append((x0, y0, z_low))

        # segments (t0, duration, p0, p1): a move to each waypoint, then a hold there
        self.segments = []
        t, p = 0.0, np.array([x0, y0, z_low])
        for wp in waypoints:
            p1 = np.array(wp, float)
            dur = max(_move_time(float(np.linalg.norm(p1 - p)), v_max, a_max), MIN_MOVE_S)
            self.segments.append((t, dur, p, p1))
            t += dur
            self.segments.append((t, hold_s, p1, p1))
            t += hold_s
            p = p1
        self.total_time = t

    def trajectory_begin(self, time):
        self.traj_enable = True
        self.start_time = time

    def initial_yaw(self):
        """Constant heading, also held before the start."""
        return self.yaw

    def _hold(self):
        return self.x0, self.y0, self.z0, 0, 0, 0, 0, 0, 0, self.yaw

    def calculate_trajectory(self, time):
        if not self.traj_enable:
            return self._hold()

        elapsed_time = time - self.start_time
        if elapsed_time > self.total_time:
            self.traj_enable = False
            return self._hold()

        t0, dur, p0, p1 = next((sg for sg in self.segments if elapsed_time < sg[0] + sg[1]), self.segments[-1])
        b, db, ddb = _min_jerk((elapsed_time - t0) / dur)
        d = p1 - p0
        p = p0 + b * d
        v = (db / dur) * d
        a = (ddb / dur**2) * d

        return (float(p[0]), float(p[1]), float(p[2]),
                float(v[0]), float(v[1]), float(v[2]),
                float(a[0]), float(a[1]), float(a[2]),
                self.yaw)


def main():
    import matplotlib.pyplot as plt  # only for the plot
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers the 3d projection)
    traj = SquareTraj()
    traj.trajectory_begin(0)

    times = np.linspace(0, traj.total_time, 2000)
    out = np.array([traj.calculate_trajectory(t) for t in times])
    x, y, z, vx, vy, vz, ax, ay, az, yaw = out.T
    tilt = np.degrees(np.arctan2(np.hypot(ax, ay), 9.81 + az))
    print(f"duration {traj.total_time:.1f} s, max speed {np.hypot(np.hypot(vx, vy), vz).max():.2f} m/s, "
          f"max |a| {np.sqrt(ax**2 + ay**2 + az**2).max():.2f} m/s^2, max tilt {tilt.max():.1f} deg")

    fig = plt.figure(figsize=(18, 10))
    ax1 = fig.add_subplot(231, projection='3d')
    ax1.plot(x, y, z, 'b-', linewidth=2)
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
    ax1.set_title('Square laps')
    ax1.set_aspect('equal')

    ax2 = fig.add_subplot(232)
    ax2.plot(x, y, 'r-')
    ax2.set_aspect('equal'); ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_title('XY projection')

    ax3 = fig.add_subplot(233)
    ax3.plot(times, z, 'b-')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Height (m)'); ax3.set_title('Z position'); ax3.grid(True)

    ax4 = fig.add_subplot(234)
    for s, c, n in ((vx, 'r-', 'Vx'), (vy, 'g-', 'Vy'), (vz, 'b-', 'Vz')):
        ax4.plot(times, s, c, label=n)
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Velocity (m/s)'); ax4.set_title('Velocity'); ax4.legend()

    ax5 = fig.add_subplot(235)
    for s, c, n in ((ax, 'r-', 'Ax'), (ay, 'g-', 'Ay'), (az, 'b-', 'Az')):
        ax5.plot(times, s, c, label=n)
    ax5.set_xlabel('Time (s)'); ax5.set_ylabel('Acceleration (m/s²)'); ax5.set_title('Acceleration'); ax5.legend()

    ax6 = fig.add_subplot(236)
    ax6.plot(times, tilt, 'k-')
    ax6.set_xlabel('Time (s)'); ax6.set_ylabel('Tilt (deg)'); ax6.set_title('Tilt needed (approx.)'); ax6.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
