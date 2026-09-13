# Calibrated MAVLink arm-angle publisher

`morphing_arm_publisher` connects `vehicle_command` to `morphing_arm_state`.
It observes `VEHICLE_CMD_DO_SET_ACTUATOR` with `param7=0`. Parameters 1–4 are
explicitly assigned to **FR, RR, RL, FL**. It does not drive PWM itself or send
command acknowledgements: existing PX4 `FunctionActuatorSet` drives outputs
assigned to Peripheral via Actuator Set 1–4. RC AUX is a different path and is
not covered.

## Configuration before activation

The module is built for `hkust_nxt-dual_default` and disabled by default.
Board startup starts it only when `MORPH_PUB_EN=1`; startup also requires
`CA_AIRFRAME=13`, `CA_ROTOR_COUNT=4`, and valid calibration.

For each arm, measure its angle from CAD neutral at normalized commands -1,
0 and +1, after configuring the output pulse range, direction and linkage:

| Arm | Angle at -1 | Angle at 0 | Angle at +1 |
| --- | --- | --- | --- |
| Front right | `MARM_FR_NEG` | `MARM_FR_ZERO` | `MARM_FR_POS` |
| Rear right | `MARM_RR_NEG` | `MARM_RR_ZERO` | `MARM_RR_POS` |
| Rear left | `MARM_RL_NEG` | `MARM_RL_ZERO` | `MARM_RL_POS` |
| Front left | `MARM_FL_NEG` | `MARM_FL_ZERO` | `MARM_FL_POS` |

Calibration parameters are measured **degrees**; output messages are **radians**.
Each curve must be strictly increasing or strictly decreasing and contained in
[-30,30] degrees. Defaults are all zero and deliberately fail validation.
Two linear segments interpolate the measured curve; verify intermediate points
on the real linkage. More strongly nonlinear linkages need a richer calibration.
Do not sweep an uncalibrated installed servo to -1/+1: first establish an output
pulse range that stays inside the arm's mechanical limits.

Enable only after all four arms are calibrated and the assigned peripheral
outputs match this order. With the aircraft disarmed and propellers removed:

```sh
param set MORPH_PUB_EN 1
morphing_arm_publisher start
morphing_arm_publisher status
```

If already running, stop it first. Restart after changing calibration or timeout;
these are read at startup. `MORPH_PUB_EN=0` disables publication from a running
instance. Use `param save` to retain settings for board startup. Changing aircraft
geometry/output configuration should be done disarmed with the publisher stopped.
The simulated `control_allocator morphing_test` command is blocked while
`MORPH_PUB_EN=1` to avoid two producers claiming different arm positions.

## Command handling

- Set parameters 5/6 to NaN unless intentionally commanding other peripherals.
- Finite normalized values must lie within [-1,1]. A NaN retains that channel's
  previous target. Infinity or any out-of-range value rejects the whole command and requires
  all four targets again because the independent output driver may have applied
  other fields.
- No state is published until all four targets have been supplied. They may
  arrive together or as partial updates. No physical neutral is invented.
- The message uses radians in FR/RR/RL/FL order, positive CAD +Z, not motor order.
  Calibrated zero need not equal zero radians.
- Only fresh commands timestamped after startup are accepted; duplicate/older,
  future-dated and expired commands are rejected. Publications use strictly
  increasing FC monotonic timestamps, even if several commands are drained in
  the same work-queue run. All-NaN commands do not refresh freshness.
- The command queue is drained at 100 Hz on `lp_default`, up to eight commands
  per run. A detected queue overrun invalidates remembered channel availability;
  all four targets must be supplied again. Prefer sending full four-arm targets
  periodically at a modest rate instead of relying only on partial updates.
- `MORPH_PUB_TMO` is the timeout in seconds (default 1). Once expired, the module
  warns, sets `stale`, and requires fresh initialization of all four channels.
  Partial initialization also expires if updates stop. It does not republish an
  old target with a fresh timestamp and does not command any physical fallback.

## Verification and limitations

Send calibrated commands using your MAVLink sender, then inspect:

```sh
morphing_arm_publisher status
listener morphing_publisher_status -n 1
listener morphing_arm_state -n 1
listener morphing_geometry -n 1
listener morphing_allocation_matrix -n 1
```

Expect `calibration_valid=true`, `initialized_mask=15`, `stale=false`, and an
increasing publication counter. `morphing_geometry.input_result=1` means accepted;
check its angles/rotor positions and the assigned matrix against the command.
Stop sending commands past the timeout and verify `stale=true`, mask zero, and
no new arm-state publication; then resend all four targets. Default ULog topics
include `morphing_publisher_status`. Diagnostics are published at 10 Hz while the
module is running; a dead/stopped module also stops these diagnostics.

These are commanded, assumed positions. Servo travel time, position feedback,
flight validation, RC AUX bridging and an arming interlock are not implemented.
The allocator still holds its last accepted geometry if this module becomes
stale or stops; no independent watchdog or physical failsafe is added.

This bridge does not control whether `FunctionActuatorSet` accepts a command or
whether PWM is actually enabled. In particular, that existing output path rounds
the group selector and ignores nonfinite fields individually; this bridge
requires exactly group zero and rejects malformed arm commands as a whole.
The sender must follow the valid interface, and output assignments/calibration
must remain consistent. Output tests, manual overrides and safety/failsafe output
changes are not observed by this bridge. A valid publication is not confirmation
that the physical servos reached their targets or permission to fly.

## Automated checks

Build and run `functional-MorphingArmPublisher` in the PX4 SITL test build.
Tests exercise calibration/startup gates, reversed/asymmetric interpolation,
partial/NaN updates, invalid inputs, replay rejection, monotonic publication,
timeout/reinitialization, and real uORB commands through to the morphing
provider's effectiveness matrix. Test calibrations are synthetic, not measurements
for this aircraft. Also run the four existing morphing allocator regression suites.
