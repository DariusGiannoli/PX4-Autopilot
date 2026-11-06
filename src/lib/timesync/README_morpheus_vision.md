# Private Morpheus external-vision clock candidate

This change fixes timestamp conversion for the existing onboard IMU + mocap
pipeline. It does not change controller gains, the EKF equations, actuator
commands, arming behavior, policy rate, calibration, or any legacy conversion
caller. It has not been flashed or flight-qualified by this offline work.

## Why the existing filter is unsuitable here

The deployed exponential filter predicts a skew in microseconds **per accepted
exchange**, while radio RTT rejection changes that cadence. In the November 6
recordings the accepted cadence changes from 4.872 to 21.033 Hz. The retained
-18.906 us/update consequently changes physical meaning from approximately
-92 to -398 us/s. An 89.557 s interval without accepted updates also freezes the
offset while the host and FC clocks continue drifting. Exact source recurrence
reproduces the observed native estimates within approximately 0.6 us; pose
conversion applies those estimates exactly. The second recording swings from
about +12 to -5.6 ms despite monotonic source timestamps and healthy independent
clock mapping.

Making the derivative time-aware improves these traces but does not establish a
safe initial rate or long holdover. The candidate therefore uses causal bounds,
with no estimated initial derivative and no equal-path-delay assumption.

## Strict path and scope

Only ODOMETRY from 245:191 with estimator VISION, frame LOCAL_FRD and
child BODY_FRD uses the strict clock. The existing bench and flight programs use
this identity. Disarmed configuration housekeeping uses 246:191; both clock peers
use the same host CLOCK_MONOTONIC epoch on the same MAVLink instance. These IDs
are a private protocol convention, not authentication. Other sources, frames,
ATT_POS_MOCAP, VISION_POSITION_ESTIMATE, landing targets and trajectories retain
the existing conversion. Their legacy 500-sample filter is unchanged.

When this strict clock is unready, stale, inconsistent or unable to bound the
capture timestamp, the receiver returns before publishing external vision. It
never substitutes receipt time. Existing feedback checks subsequently observe
missing fresh aiding; the existing MHE1 wire protocol is unchanged.

## Bounds and assumptions

For an FC-origin exchange, let A be the actual originate timestamp, R the remote
reply stamp, and B the actual FC receipt timestamp, all in microseconds. With
correct stamps and nonnegative transport delay, FC-minus-remote offset at the
reply event lies in [A-R, B-R]. The implementation expands for relative drift
across the RTT and for nanosecond-to-microsecond quantization, then intersects
recent intervals expanded to the current FC time.

Defaults match the existing bounded host-clock assumptions:

| Setting | Value |
|---|---:|
| Maximum recorded exchange RTT | 20 ms |
| Relative drift bound | 100 ppm |
| Evidence window | 2 s |
| Ring capacity | 128 exchanges |
| Minimum fresh samples | 8 |
| Maximum uncertainty half-width, including capture-time expansion | 5 ms |

These are explicit engineering assumptions and screening limits, not a derived
RL stability margin. Longer RTTs are rejected; lower RTT is not equated with
symmetric transport. Capture-time conversion also covers the unknown local
capture instant: it uses the conservative factor rho/(1-rho), equal to 1/9999 at
100 ppm. The existing strict 6 ms independent timing-proof limit is unchanged.

Future originates, duplicate/reordered older originates and invalid timestamps
are rejected. Overlapping exchanges can legitimately reorder their remote
stamps; only causally nonoverlapping exchanges establish definite remote event
order. The current MAVLink stream has no outstanding-token registry shared with
the receiver: a syntactically plausible unsolicited echo cannot be authenticated
by this class. Correct exchange timestamps and the private trusted-peer
convention remain explicit assumptions.

## Lifecycle and diagnostics

Ordinary expiry clears samples and readiness and advances a generation. No
timestamp can be converted from the stale estimate. At least 8 fresh bounded
exchanges must establish readiness again, without rebooting the FC. A
drift-expanded last-valid interval can detect an impossible new epoch across a
gap, but it never grants readiness or supplies a holdover timestamp.

Contradictory intervals, definite clock reversal and arithmetic faults latch.
They require explicit reconstruction of the MAVLink instance/FC clock object
and a new timing proof; resetting the unrelated legacy exponential filter does
not erase that evidence. A clock change small enough to remain inside all valid
uncertainty/drift intervals is mathematically indistinguishable from allowed
drift; no stronger claim is made.

TimesyncStatus retains all original fields and adds strict-clock readiness,
fault, sample count, offset, uncertainty, age and generation. Like the original
topic, it publishes on received TIMESYNC replies. Without replies, its last
published readiness is historical: compare its timestamp/age to current FC time.
The receiver still checks current readiness on every strict vision packet and
drops stale packets. No fake exchange samples are published for diagnostics.

## Reproducible offline validation

Use Morpheus's Python environment, which contains numpy and pyulog:

```sh
/home/dg/dev/lis/repos/Morpheus/.venv/bin/python \
  src/lib/timesync/test/run_native_tests.py \
  --output /tmp/vision_clock_native_result \
  --session-root /home/dg/dev/lis/repos/Morpheus/control/logs/morpheus_estimator/ps5_estimator_20261105T160951_038754Z
```

The harness compiles actual production Timesync.cpp and BoundedVisionClock.cpp
with C++14, warnings-as-errors, AddressSanitizer and UndefinedBehaviorSanitizer.
Only hrt and uORB publication use deterministic native adapters. Synthetic checks
cover signed offsets, ±100 ppm drift, cadence changes, asymmetric delay, broad
uncertainty, RTT rejection, duplicate/future echoes, overlap/reordering, ordinary
idle/reacquisition, hard-fault recovery, strict rejection and legacy behavior.
The source contract verifies that the real receiver rejects before publication
and retains all four other legacy callers. A separate full board build is also
required to compile the actual MAVLink translation unit and check flash/RAM.

Recorded replays use exact matched FC-origin exchanges from the closed ULogs and
raw host replies, without synthesizing a learned initial skew. Candidate outputs
are compared with the ORIGINAL independently mapped capture timestamps and
uncertainties. Old failed proofs remain failed. Each result includes input,
source and compiled-binary hashes, native commands, raw native output and summary.
The combined replay carries one object through the two actual November6 windows
and the observed idle, including a labelled synthetic midgap query that must
drop. Counterfactual replay is not a new physical timing proof, EKF replay,
control-stability test or flight qualification.

The standalone clock occupies 3152 bytes per object on the native 64-bit compiler
(check the harness output for the exact current size); actual target RAM and
flash are checked by the board build. No dynamic allocation occurs in the clock.
