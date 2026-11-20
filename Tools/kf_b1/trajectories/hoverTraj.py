import numpy as np

# ---------------------------------------------------------------- parameters (z up, metres, seconds, radians)
X0, Y0, Z0 = 0.0, 0.0, 0.6   # hover point
HOVER_S = 20.0               # hover duration after trajectory_begin
YAW = 0.0                    # constant heading


class HoverTraj():
    def __init__(self, x0=X0, y0=Y0, z0=Z0, hover_s=HOVER_S, yaw=YAW):
        """
        Hover at a fixed point with constant yaw (the hover pulses are added by the PX4 module).

        :param x0, y0, z0: Hover point
        :param hover_s: Hover duration (s)
        :param yaw: Constant heading (rad)
        """
        self.x0 = x0
        self.y0 = y0
        self.z0 = z0
        self.yaw = yaw
        self.total_time = hover_s
        self.start_time = 0
        self.traj_enable = False

    def trajectory_begin(self, time):
        self.traj_enable = True
        self.start_time = time

    def initial_yaw(self):
        """Constant heading, also held before the start."""
        return self.yaw

    def calculate_trajectory(self, time):
        if self.traj_enable and time - self.start_time > self.total_time:
            self.traj_enable = False
        return self.x0, self.y0, self.z0, 0, 0, 0, 0, 0, 0, self.yaw


def main():
    import matplotlib.pyplot as plt  # only for the plot
    traj = HoverTraj()
    traj.trajectory_begin(0)
    times = np.linspace(0, traj.total_time, 200)
    out = np.array([traj.calculate_trajectory(t) for t in times])
    print(f"duration {traj.total_time:.1f} s, hover at ({traj.x0}, {traj.y0}, {traj.z0}) m, yaw {traj.yaw} rad")

    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for k, c, n in ((0, 'r-', 'x'), (1, 'g-', 'y'), (2, 'b-', 'z')):
        axs[0].plot(times, out[:, k], c, label=n)
    axs[0].set_ylabel('Position (m)'); axs[0].legend(); axs[0].grid(True); axs[0].set_title('Hover')
    axs[1].plot(times, np.degrees(out[:, 9]), 'k-')
    axs[1].set_ylabel('Yaw (deg)'); axs[1].set_xlabel('Time (s)'); axs[1].grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
