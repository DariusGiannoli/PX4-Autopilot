# kf_b1: online B1 estimation with hover pulses

Two PX4 modules estimate the control-effectiveness matrix B1 (Δν = B1·Δu, ν = [a_z, α_x, α_y, α_z]) in flight:
`kf_b1` (Kalman filter, estimator only) and `throw_sysid` (hover pulses on top of the controller's motor commands).
B1 rows: a_z, α_x, α_y, α_z (FLU). B1 columns: motors FL, RL, RR, FR.

## 1. Scripts (`Tools/kf_b1/`)

Python 3 with `numpy`; `pymavlink` for the tools; `matplotlib` only for the trajectory plots.

| Script | What it does |
|---|---|
| `trajectories/hoverTraj.py` | Hover at a fixed point, constant yaw; parameters at the top of the file (point, duration, yaw) |
| `trajectories/squareTraj.py` | Square laps, corners at alternating heights, constant yaw, stop at each corner; parameters at the top of the file (side, heights, laps, speed, acceleration, hold) |
| `b1_watch.py` | Live B1 estimate over MAVLink; optional comparison with a reference B1 |
| `hover_pulses.py` | Sets parameters, starts `throw_sysid`, fires the pulses on Enter (or `--after-airborne S`), prints the fit; never arms, takes off or lands |
| `sitl/sitl_controller.py` | SITL only: external controller stand-in; trajectory file → cascaded PID → body rates + thrust over MAVLink (OFFBOARD); Enter = `trajectory_begin` |
| `sitl/gz_params.sh` | SITL only: parameters for the Gazebo x500 (mocap-only EKF2, thrust curve, motor lag, prior ~15 % off, logging) |

Both trajectories: same interface as `EightTraj` (`trajectory_begin(t)`, `initial_yaw()`, `calculate_trajectory(t)` →
x, y, z, vx, vy, vz, ax, ay, az, yaw), z up, hold point before the start and after the end.
`python3 trajectories/squareTraj.py`: plot, peak speed, acceleration, tilt.

## 2. PX4 changes

| Change | What it does |
|---|---|
| `src/modules/kf_b1/` (new) | Kalman filter estimate of B1 from `vehicle_angular_velocity` (angular acceleration), `vehicle_acceleration` and `actuator_motors`. Publishes `b1_estimate` and a MAVLink `DEBUG_FLOAT_ARRAY` named `kf_b1`. Fits B1 by least squares on the hover pulses, then restarts the filter from that fit. Commands nothing. |
| `src/modules/throw_sysid/` (new) | Hover pulses: on `throw_sysid arm`, +amplitude then −amplitude on one motor at a time, motor by motor. Amplitude and duration are parameters (`THRSID_AMP`, `THRSID_SLOT`), not fixed. Publishes the offsets in `throw_sysid_status`. No mode change. |
| `src/modules/control_allocator/` (12 lines) | Adds the pulse offsets to its motor commands, then clips. |
| `msg/B1Estimate.msg`, `msg/ThrowSysidStatus.msg` (new) | The two new topics. |
| `ROMFS/px4fmu_common/init.d/rc.mc_apps` | Starts `kf_b1` at boot (`KFB1_EN`). `throw_sysid` is started by hand or by `hover_pulses.py`. |
| `src/modules/logger/logged_topics.cpp` | Logs `b1_estimate`, `throw_sysid_status` and `vehicle_acceleration` at full rate with the high-rate logging profile (`SDLOG_PROFILE` 17 = default + high rate). |
| `boards/px4/sitl/default.px4board` | Both modules enabled in SITL. For the drone's board: `CONFIG_MODULES_KF_B1=y`, `CONFIG_MODULES_THROW_SYSID=y` in its `.px4board`. |
| `Tools/kf_b1/` (new) | The scripts above. |

The pulses go through `control_allocator`, so they work when the controller sends attitude + thrust or body rates +
thrust setpoints, not when it sends motor commands directly.

## 3. Parameters

Suffix `*` = row: 0 a_z [m/s²], 1 α_x, 2 α_y, 3 α_z [rad/s²]. Values in a row are listed in that order.
*Applied*: live = within ~1 s; arming = at the next arming; arm cmd = at the next `throw_sysid arm`; reboot.

### Kalman filter (`kf_b1`)

| Parameter | Default | Applied | Meaning |
|---|---|---|---|
| `KFB1_EN` | 1 | reboot | Start `kf_b1` at boot |
| `KFB1_DT_US` | 2000 µs | live | Minimum time between filter steps |
| `KFB1_R*` | 0.2 / 10 / 10 / 2 | live | Measurement noise std per row |
| `KFB1_Q*` | 0.005 / 0.5 / 0.5 / 0.05 | live | Random-walk std per row and 2 ms step (how fast B1 may change) |
| `KFB1_P0_*` | 0.5 / 100 / 100 / 10 | arming | Prior std of each entry per row (trust in the prior) |
| `KFB1_PCAP` | 3 | live | Covariance stops growing at this × `KFB1_P0_*` |
| `KFB1_LPF_HZ` | 5 Hz | live | Identification low-pass on both ν and u, in the filter and the pulse fit (0 = off) |
| `KFB1_TAU_M` | 0.01 s | live | Motor lag model (first order) on the command; 10 ms identified for the test drone (0 = none) |

### Prior B1 (`kf_b1`)

| Parameter | Default | Applied | Meaning |
|---|---|---|---|
| `KFB1_G0_rc` (16) | row 0: 12, 12, 12, 12 · row 1: 350, 350, −350, −350 · row 2: −350, 350, 350, −350 · row 3: 40, −40, 40, −40 | arming | Prior B1 entry, row r (0-3 as above), column c (0 FL, 1 RL, 2 RR, 3 FR) |

### Hover pulses (`throw_sysid`) and the fit on them (`kf_b1`)

| Parameter | Default | Applied | Meaning |
|---|---|---|---|
| `THRSID_AMP` | 0.08 | arm cmd | Pulse amplitude added to the motor command (keep below the hover command; warning otherwise) |
| `THRSID_SLOT` | 0.5 s | arm cmd | Time per motor (half +, half −); total 4 × slot = 2 s; range 0.02-1 s |
| `THRSID_RATE_LIM` | 20 rad/s | arm cmd | Body-rate budget: a motor's pulse is cut when a rate exceeds its share |
| `THRSID_RATE_MAX` | 30 rad/s | live | Body rate that stops the pulses |
| `THRSID_TILT_MAX` | 60° | live | Tilt that stops the pulses |
| `THRSID_MAX_DROP` | 1 m | live | Height loss that stops the pulses |
| `KFB1_THR_NMIN` | 40 | live | Minimum samples for the fit to be used |
| `KFB1_THR_STD` | 0.2 | live | Trust in the fit at the filter restart: each B1 entry known to ± this fraction of its row's gain |
| `THR_MDL_FAC` (PX4) | 0 | live | Thrust curve of the motor output; 0 = command linear in thrust |

### Shell examples (`pxh>` in SITL, MAVLink shell on the drone)

| Command | Does |
|---|---|
| `param show KFB1*` / `param show THRSID*` | List all parameters of a module with current values |
| `param set THRSID_SLOT 0.3` | Shorter pulses: 0.3 s per motor, 1.2 s in total |
| `param set THRSID_AMP 0.06` | Smaller pulse amplitude |
| `param set KFB1_G0_11 380` | One prior entry (row 1 α_x, column 1 RL) |
| `param reset KFB1_G0_*` | All prior entries back to the compiled defaults |
| `param save` | Store now (`set` / `reset` also store automatically) |
| `kf_b1 status` | Current B1, steps since arming, last pulse fit, prior or fit as start |
| `throw_sysid start` / `arm` / `cancel` / `status` | Start the module / fire the pulses / stop them / phase and settings |

## 4. Test procedure

`b1_watch.py` and `hover_pulses.py` each on their own MAVLink link, separate from the controller's (`<port1>`, `<port2>`).

1. Bench, props off: `kf_b1 status` shows the module running and the prior.
2. On the PC, before takeoff:
   ```bash
   python3 Tools/kf_b1/b1_watch.py --url udpin:0.0.0.0:<port1> --ref testdrone --ref-volt <battery V>
   python3 Tools/kf_b1/hover_pulses.py --url udpin:0.0.0.0:<port2> [--param THRSID_SLOT=0.3]
   ```
3. Take off and hover with the controller (`hoverTraj`, or the hold before the trajectory starts).
4. Enter in `hover_pulses.py`: pulses (2 s), fit, `phase DONE`. `Ctrl-C` cancels.
5. Start the trajectory (`squareTraj`), watch the estimate.
6. Land, copy the `.ulg` log.

## 5. SITL test (Gazebo)

Pulses on top of an external controller (body rates + thrust), all over MAVLink. Gazebo x500 with external vision =
mocap stand-in. Needs Gazebo Garden (PX4 v1.14) and `make px4_sitl` built once.

First time only: start PX4 (terminal 1 below), run `Tools/kf_b1/sitl/gz_params.sh` in a second terminal, then
`shutdown` in terminal 1 and start PX4 again.

| Terminal | Command (from the PX4 root) |
|---|---|
| 1. PX4 + Gazebo | `cd build/px4_sitl_default && PX4_SYS_AUTOSTART=4005 PX4_SIM_MODEL=gz_x500_vision ./bin/px4 -i 1` |
| 2. Live B1 | `python3 Tools/kf_b1/b1_watch.py --url udpin:0.0.0.0:14031 --ref x500lin` |
| 3. Pulses | `python3 Tools/kf_b1/hover_pulses.py --url udpin:0.0.0.0:14550` |
| 4. Controller | `cd Tools/kf_b1/sitl && python3 sitl_controller.py --traj ../trajectories/hoverTraj.py --arm` (or `squareTraj.py`) |

1. Terminal 4: arms, takes off to the trajectory's start point and holds.
2. Enter in terminal 3: pulses (2 s), fit, `phase DONE`; terminal 2 switches to "started from throw".
3. Enter in terminal 4: `trajectory_begin`; after the trajectory, 2 s hold, then AUTO.LAND.

x500 reference (`x500lin`, `THR_MDL_FAC` 0.67): own gains 4.15 / 68.7 / 68.7 / 3.42. Gazebo result (square, 0.5 s pulses):
fit 0 / −9 / −10 / −11 %, estimate at the end of the square +2 / −9 / −10 / −10 %.

## 6. Expected values

From the drone model (T_i/m = 12.68·u_i + 0.183, u_i = command × V/16.8), own gains at 16.8 V: thrust 12.68, roll 401,
pitch 377, yaw 51; they scale with V/16.8 (at 15.2 V: 11.5 / 363 / 341 / 46). `b1_watch.py --ref testdrone --ref-volt V`
compares against these.
