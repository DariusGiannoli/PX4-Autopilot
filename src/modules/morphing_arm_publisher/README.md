# Calibrated MAVLink arm-angle publisher

`morphing_arm_publisher` connects `vehicle_command` to `morphing_arm_state`.
It observes `VEHICLE_CMD_DO_SET_ACTUATOR` with `param7=0`. Parameters 1–4 are
assigned according to `MARM_MODEL`: legacy **FR, RR, RL, FL** or measured **FR, RL, FL, RR**. It does not drive PWM itself or send
command acknowledgements: existing PX4 `FunctionActuatorSet` drives outputs
assigned to Peripheral via Actuator Set 1–4. RC AUX is a different path and is
not covered.

## Current measured RR model (calibration 1210834)

Select `MARM_MODEL=1` with matching host and firmware calibration ID `1210834`.
The source in Morpheus is
`dynamics/arm_mapping/candidates/rr_main5_set3_1210834.json`. Verify the generated
C++ table from the Morpheus root with:

```sh
python3 control/tools/export_arm_calibration.py --check \
  --source dynamics/arm_mapping/candidates/rr_main5_set3_1210834.json
```

The matching host output profile is
`control/config/arm_validation/output_profiles/main5_set3_1210834/`.
Physical wiring remains MAIN5 RR, MAIN6 RL, MAIN7 FR, MAIN8 FL. Actuator-set
routing changed from the older 1210832/1210833 profiles:

| Actuator set / command parameter | Arm | MAIN | Min / neutral / max (us) |
| --- | --- | --- | --- |
| 1 | RL | 6 | 1150 / 1500 / 1750 |
| 2 | FR | 7 | 1150 / 1400 / 1750 |
| 3 | RR | 5 | 1150 / 1500 / 1750 |
| 4 | FL | 8 | 1150 / 1500 / 1750 |

MAIN 5–6 and MAIN 7–8 PWM timer groups must match at either 50 Hz or 200 Hz.
The current host output profile requires 200 Hz. Accepting that configuration
in firmware does not establish the transient model's accuracy at that rate.
PWM reversal must be disabled, and each output assignment and disarmed value
must match the compiled contract.

The shared static hysteresis curves now come from the measured RR sweep, with
a 1500-us reference neutral. FR/RR/RL/FL neutral values are
1400/1500/1500/1500 us; the firmware removes each arm's neutral offset before
evaluating the RR reference curves. The dynamic response parameters are retained
provisionally from the earlier FR fit; they were not independently identified
for every installed arm.

The host inverts the generalized-play/hysteresis map separately for each arm.
Commands remain normalized raw PWM: physical PWM is `1450 + 300 * command`,
rounded to whole microseconds. Commands are not angle fractions. The publisher
uses the compiled actuator-set routing above, reconstructs four independent
hysteresis states, and integrates the speed-limited second-order response with
steps <=2 ms. It publishes estimated current angles at 100 Hz in canonical
FR/RR/RL/FL order. `MARM_*_NEG/ZERO/POS/CTR` are ignored in measured mode;
`MARM_*_DIR` selects each arm's increasing-PWM to CAD angle sign.

The first accepted packet must contain all four configured neutral PWM values,
and the physical arms must already be at neutral. Subsequent packets must
contain all four finite targets. Repeated identical PWM commands do not reapply
hysteresis. On timeout, queue overflow, invalid command or >100 ms scheduler gap,
the model stops refreshing geometry and requires publisher restart from neutral.
The independent PWM driver may continue holding or applying commands. Lost radio
packets before `vehicle_command` publication cannot be detected here and can
desynchronize host and onboard history.

The common host command limit remains +/-20 degrees; the separate RR range
workflow uses its explicitly configured asymmetric range. The allocator and
publisher support a +/-35-degree prediction envelope for transient overshoot.
That envelope does not increase physical PWM bounds or establish mechanical
clearance. Out-of-range predictions latch a fault instead of clipping geometry.
Returning to zero angle can request PWM different from the starting neutral
because hysteresis compensation depends on movement history.

The retained 38-ms delay is an earlier **host-command to mocap-receipt delay**,
not an isolated servo delay or a measured RR radio delay. It is excluded from
onboard prediction. These are empirical angle predictions without servo-position
feedback, an arming interlock or independent flight qualification. The allocator
holds its last accepted geometry if the publisher stops.

## Legacy model (`MARM_MODEL=0`)

The following sections describe the original three-point mode.

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

## Non-midpoint neutral

MARM_FR_CTR, MARM_RR_CTR, MARM_RL_CTR, MARM_FL_CTR select the normalized
command for the corresponding ZERO angle sample. Their default 0 preserves
the previous behavior. NEG/POS are still samples at commands -1/+1; interpolation
is piecewise through the configured center. Centers must be finite and strictly
inside (-1,1). Restart after parameter changes. ZERO=0 makes the center the CAD
neutral pose. Endpoint angles remain measured calibration inputs.

For the 2026-09-13 screenshot, Sets 1/2/3/4 are MAIN 8/7/6/5. With PWM
1000..2000, no reversal, and DIS 1500/1420/1550/1500 in set order, use
CTR=[0,-0.16,0.10,0] and ZERO=[0,0,0,0]. The sender must use those same centers.
This output assignment does not establish the actual physical arm identity.
