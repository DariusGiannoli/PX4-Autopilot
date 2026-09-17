# Morphing quadrotor geometry and allocator integration

Implemented in `src/modules/control_allocator/ActuatorEffectiveness/MorphingQuadGeometry.hpp`.
This is a pure, header-only C++14 geometry function. It has no heap allocation,
PX4 runtime dependencies, parameter writes, or hardware side effects. The new
`ActuatorEffectivenessMorphingQuad` provider connects it to the existing allocator.
The provider is registered in the build and selected explicitly with
`CA_AIRFRAME=13`; no board or existing airframe default is changed.

## Allocator integration

The provider inherits the multirotor's sequential-desaturation preference and RPY
normalization. It creates four **motor** columns and no servo columns. Its only
input from servo-driving code is the `morphing_arm_state` uORB topic.
The core provider does not drive servos. A separate opt-in MAVLink publisher is
now available; see [its calibration and setup guide](../../src/modules/morphing_arm_publisher/README.md).
It adds no arming interlock or position feedback.

- `MorphingArmState.msg` carries accepted angles in radians in FR/RR/RL/FL order.
  Its producer must use those same angles for servo targets. It is an internal
  state topic, not a raw radio-command interface or measured-position report.
- Before any valid state, the provider uses neutral geometry. Newer timestamps
  with valid angles update geometry; repeated, older, or invalid states do not
  replace the last accepted geometry. No timeout-induced arm movement is added
  here; initialization and communication-loss policy belong to servo control.
- Ordinary geometry rebuilds are limited to 100 Hz during allocation runs;
  unchanged angles do not trigger matrix recomputation. Topic polling may be
  faster once the interval has elapsed. This is not a separate
  100 Hz task and does not determine PWM frequency. Other effectiveness models
  keep their previous 100 ms interval.
- `CA_ROTOR_COUNT` must be 4. Existing `CA_ROTOR0..3_CT/KM` supply coefficients in
  PX4 motor order (FR/RL/FL/RR). Rotor position and axis parameters are ignored by
  this provider: the CAD geometry and vertical thrust axes define them.
- `CA_MORPH_COGX/Y/Z` specify a fixed CoG offset in FRD metres, initially zero.
  There is no runtime mass-distribution estimator.
- Configuration updates compute allocation normalization using **neutral**
  geometry with current coefficients and CoG. Motion updates invert the current
  geometry while retaining that scale. This also works when configured while
  folded. Existing actuator-failure handling continues to freeze normalization.
- The provider uses a numerical row cutoff of 1e-6 instead of the generic 0.05
  physical-scale-inappropriate cutoff; all other models retain 0.05. This keeps
  valid small yaw coefficients. It does not manufacture authority in degenerate
  geometries or replace configuration/flight validation.

Core changes consist of three optional provider policies (update interval,
authority threshold, neutral normalization reference), one optional normalization
setter in the allocator, and source selection. Defaults preserve standard-model
behavior. Motor publication, clipping, slew limiting, failure-column removal,
and the sequential-desaturation algorithm are reused.

The listener is implemented inside PX4's actuator-effectiveness provider; no
separate scheduled script is needed. The separate `morphing_arm_publisher` observes calibrated MAVLink actuator
commands; the provider remains independent of the command transport.

## Interface

```cpp
bool morphing_quad::compute(
    const RotorValues &angles, // radians, CAD joint convention
    const RotorValues &ct,     // nonnegative thrust/effectiveness coefficients
    const RotorValues &km,     // signed PX4 reaction-torque/thrust ratios
    const Vector3 &cog,        // PX4 FRD metres relative to CAD origin
    GeometryResult &result);
```

All three input arrays use the existing Python model's order:
**front right, rear right, rear left, front left**. The caller must explicitly
provide coefficients and a CoG; neither propeller spin directions nor propulsion
calibration is inferred from geometry. For the initial fixed-centre approximation,
pass `{0.f, 0.f, 0.f}` as CoG. This does not claim that the assembled aircraft's
actual centre of gravity is at the model origin.

Angles are offsets from the CAD neutral configuration, limited to ±pi/6
(±30 degrees). Positive means rotation about CAD +Z, counterclockwise viewed
from above, for every arm. This is not an "inward-positive" convention: folding
opposite arms symmetrically may require opposite signs. The input limit overrides
the URDF's ±45 degree limits. Degrees must be converted to radians upstream.

Output positions and matrix columns use standard PX4 quad motor order:

| PX4 motor/column | Name | Input index |
| --- | --- | --- |
| 1 / 0 | Front right | 0 |
| 2 / 1 | Rear left | 2 |
| 3 / 2 | Front left | 3 |
| 4 / 3 | Rear right | 1 |

Physical PWM channel assignments remain independent of these logical motor numbers.

`rotor_position_body` contains thrust-site positions relative to the CAD origin,
in metres. `effectiveness` is a 6x4 matrix with rows
`[torque_x, torque_y, torque_z, thrust_x, thrust_y, thrust_z]`; torque is about the
supplied CoG. Its columns can later be placed in PX4's 6x16 matrix.

The return value is false for nonfinite inputs, angles outside the range,
negative CT, or arithmetic overflow. On failure the output remains unchanged.
CT=0 is accepted and gives a zero-effectiveness column. The function does not
silently clamp angles: the servo-driving code must publish the angles it actually
accepted after any limiting or conversion.

## Coordinate and physical conventions

The conversion inferred from the CAD front/right link labels is:

```text
x_PX4 = -y_CAD
y_PX4 = -x_CAD
z_PX4 = -z_CAD
```

This is a proper rotation (determinant +1). CAD +Z becomes PX4 -Z. Therefore the
horizontal offset is rotated by **minus** the input angle in PX4 coordinates.
The physical FC installation must use the same chosen forward direction.

The hinge positions and neutral offsets are stored in PX4 coordinates:

```text
position = hinge + Rz(-angle) * neutral_offset
lever = position - cog
B_column = CT * [-lever_y, lever_x, KM, 0, 0, -1]
```

For upward thrust, positive PX4 KM produces positive body yaw torque. The reference
Python model uses `+spin * reaction_ratio * thrust_axis`, whereas PX4 uses
`-KM * CT * thrust_axis`; the reference comparison therefore uses
`KM = -spin * reaction_ratio`. These are algebraic sign conventions, not a claim
that the physical propellers have already been checked.

Using CT=1 maps physical rotor thrusts in newtons into physical torque/thrust.
Other CT values scale each column for actuator-effectiveness inputs. The returned
matrix is **B, not B's pseudoinverse or PX4's normalized mixing matrix**.

The assembled mass is 0.170 kg. It is not an input to this geometry calculation.
The reference CAD mass properties are used only for cross-validation of CoG
translation; they are not treated as measured assembled-aircraft properties.

## CAD provenance

Source repository: sibling `morphing-quadrotor-learning`, inspected at
`8e0197b7751ba5cab84d45960a6f67d025856bb6`.

- URDF: `robot_description/morphing_quadrotor.urdf`
- URDF SHA256: `b1f12853f90f91386f4f2aa743f5d4d0eff1900991357bec839c792d968655c6`
- Reference model: `dynamics/morphing_quadrotor.py`
- Model SHA256: `2e41a54813633bca8298d08204c49d21f6f83b29c84d042195f96029fb9105fc`

Each horizontal hinge coordinate is ±0.028 m; horizontal hinge-to-rotor distance
is 0.05640203337 m. The offset includes the rotor collision-origin thrust site
used by the Python/MuJoCo model (nominal thrust-site Z is -0.025 m in PX4).
Nanometre-scale horizontal export noise is omitted. Vertical site choice does
not affect torque from vertical thrust, but matching the reference site makes
the position comparison meaningful. CAD revisions require rechecking constants.

## Run the host verification

From the PX4 repository root, using Python 3.9+ with NumPy and a C++14 compiler:

```sh
python3 Tools/morphing_quad_geometry/test_geometry.py
```

For a different checkout location:

```sh
python3 Tools/morphing_quad_geometry/test_geometry.py --model-root /path/to/morphing-quadrotor-learning
```

`--compiler` or `CXX` selects the compiler (default `clang++`). The test builds
the actual firmware header with warnings as errors and address/undefined-behaviour
sanitizers, in a temporary directory. No reference-model files are modified.

Coverage:

- All 81 combinations of -30/0/+30 degrees across four arms.
- 1,000 deterministic random configurations within the limits.
- CAD-to-PX4 transformation, angle sign, and input-to-motor ordering.
- Fixed-origin, reference-model dynamic CoG, and arbitrary CoG offsets.
- Unequal CT values and resulting mixed motor wrenches.
- Zero lateral thrust and four-axis rank for all sampled configurations.
- Rejected invalid angles, NaN/Inf, negative CT, overflow, and unchanged output.
- Zero CT and isolated front-right arm motion.

Verified on macOS with Clang: 1,081 reference comparisons passed; maximum position
error 9.09e-9 m and maximum matrix-entry error 2.42e-7. The matrix comparison uses
relative tolerance 3e-6 and absolute tolerance 2e-7 because CT scales the entries.
The runner prints reference hashes for reproducibility. This is host validation,
not an ARM firmware build, servo calibration, or flight test.

## PX4 integration tests

`ActuatorEffectivenessMorphingQuadTest.cpp` is registered as a functional gtest.
It exercises real uORB state publication, parameter loading, provider updates,
neutral references, and the existing sequential-desaturation allocator. It also
checks invalid/replayed messages, unchanged geometry, CoG offsets, configuration
updates while folded, saturation, and failure-time normalization preservation.

Build the `functional-ActuatorEffectivenessMorphingQuad` target in a PX4
`px4_sitl_test` CMake build, then run it from the build directory. Also run
`unit-ControlAllocationPseudoInverse` and
`functional-ControlAllocationSequentialDesaturation` as regression coverage for
the shared normalization extension. This tests the actual provider/allocator;
it does not replace an ARM target build or hardware testing.

Validation on macOS: the control-allocator module and all four targeted test suites
compiled and passed (including `functional-ControlAllocatorMorphing`). This checkout required CMake 3.x rather than
CMake 4 and the local test-build flag `-Wno-error=vla-cxx-extension` for an
existing variable-length array in PX4's desktop shell. No shell/build-system
source was changed to accommodate the host compiler. The functional test disables
parameter autosave, following PX4's existing test convention, because its runner
does not start an autosave work queue.

Example from an already configured test build:

```sh
cmake --build build/morphing-tests --target functional-ControlAllocatorMorphing functional-ActuatorEffectivenessMorphingQuad unit-ControlAllocationPseudoInverse functional-ControlAllocationSequentialDesaturation modules__control_allocator
ctest --test-dir build/morphing-tests --output-on-failure --timeout 20 -R '^(functional-ControlAllocatorMorphing|functional-ActuatorEffectivenessMorphingQuad|unit-ControlAllocationPseudoInverse|functional-ControlAllocationSequentialDesaturation)$'
```

## Connecting the servo-driving code

The integration boundary is `msg/MorphingArmState.msg`: a monotonic timestamp and
four accepted angles. The provider already subscribes to it. At the point where
the servo-driving code accepts its final arm targets, publish those same targets:

```cpp
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/morphing_arm_state.h>

// Persistent member of the servo-driving class:
uORB::Publication<morphing_arm_state_s> _arm_state_pub{ORB_ID(morphing_arm_state)};

// After accepting the servo targets; values below are radians from CAD neutral:
morphing_arm_state_s state{};
state.timestamp = hrt_absolute_time();
state.angle[0] = accepted_fr_rad;
state.angle[1] = accepted_rr_rad;
state.angle[2] = accepted_rl_rad;
state.angle[3] = accepted_fl_rad;
_arm_state_pub.publish(state);
```

This is a producer-side integration example, not a second servo implementation.
A custom servo-driving source must supply the four named variables; the included
MAVLink publisher performs the corresponding calibrated conversion for group 0. If it only
exposes normalized outputs or PWM pulse widths, it must convert those values to
arm angles using its own calibration before publishing. Raw PWM values cannot be
interpreted as angles by the allocator.

Select `CA_AIRFRAME=13` and `CA_ROTOR_COUNT=4` to use this provider. Aircraft
geometry constants are in `MorphingQuadGeometry.hpp`; coefficient and CoG
parameters are described above. No physical output assignment is required to
exercise the host integration tests.

With no publisher the provider stays at neutral; after a valid update it holds
the last accepted geometry until another valid update. Publishing commanded
angles assumes those positions immediately: actual servo travel time and position
feedback are not modeled. The listener does not command servos or alter arming.

## Recorded diagnostics for bench verification

The default ULog profile now includes three optional topics. They do not activate
or command servos. The two diagnostic topics are published only by the selected
morphing provider; no console spam is added to the control loop.

| Topic | Contents |
| --- | --- |
| `morphing_arm_state` | Angles published by the servo-driving code and their source timestamp |
| `morphing_geometry` | Last examined input and its outcome; accepted angles; rotor positions; CoG; CT/KM; raw matrix; matrix generation; rejected-input count |
| `morphing_allocation_matrix` | Matrix actually assigned to the allocator after failed-motor masking and weak-row removal; generation and accepted-angle timestamp; handled failure mask |

The geometry snapshot is emitted when a new input is examined or a new matrix is
supplied. The applied-matrix snapshot is emitted only on actual matrix assignment.
`matrix_updated=true` identifies a geometry snapshot for a new matrix generation.
Join that snapshot to the applied record with the same `matrix_generation` within
one provider lifetime. The counter restarts when the provider is recreated; use
timestamps to distinguish separate runs. Repeated unchanged angles can update
`timestamp_angle` without producing a new matrix generation.

`input_result`: 0 = no input examined yet, 1 = accepted, 2 = invalid angles or
geometry configuration, 3 = non-increasing/zero timestamp. Invalid inputs do not
replace accepted angles. `geometry_valid=false` makes positions and raw matrix
NaN, rather than displaying a plausible zero matrix. `timestamp_angle=0` identifies
the default-neutral assumption before the first accepted input.

### Capturing a disarmed bench test

Use the modified firmware, a working SD card, and the default logging profile
(`SDLOG_PROFILE` includes bit 0; its normal default is 1). A custom
`/fs/microsd/etc/logging/logger_topics.txt` replaces the default topic list. If one
exists, add these lines to that file instead of assuming defaults will apply:

```text
morphing_arm_state 0
morphing_geometry 0
morphing_allocation_matrix 0
```

The interval 0 requests no additional topic downsampling; it does not guarantee
lossless capture. Check ULog dropouts and retain the normal control/output topics
in a custom list. Default profiles already include `actuator_motors`,
`actuator_outputs`, `vehicle_torque_setpoint`, `vehicle_thrust_setpoint`, and
`control_allocator_status`, at their existing logging intervals. These provide
context but are not a synchronized trace of every control-loop iteration.

In the PX4 console, with propellers removed and the aircraft disarmed:

```sh
logger status
logger on
listener morphing_geometry
listener morphing_allocation_matrix
```

Run the commands separately. `logger on` requires the logger to be running and
allows logging without arming; check `logger status` for active file logging and
the log path. If the logger is not running, this checkout supports
`logger start -e -t`. Once the angle source has published neutral and the test
movements, end recording with `logger off` and retrieve the `.ulg` file.
These instructions do not start a servo driver or supply missing angle inputs.

### What to compare

Angles use arm order **FR RR RL FL**. Positions, coefficients and matrix columns
use motor order **FR RL FL RR**. Positions are flattened as `3*motor+axis`; matrix
entries are flattened as `4*row+motor`. Rows are torque X/Y/Z, force X/Y/Z.

For each valid geometry record, independently recompute positions from `angle`
and the CAD constants in `MorphingQuadGeometry.hpp`, then check, for each motor:

```text
x = rotor_position[3*motor]   - cog[0]
y = rotor_position[3*motor+1] - cog[1]
B[0,motor] = -ct[motor] * y
B[1,motor] =  ct[motor] * x
B[2,motor] =  ct[motor] * km[motor]
B[3,motor] = 0
B[4,motor] = 0
B[5,motor] = -ct[motor]
```

Use floating-point tolerances, not text equality. The geometry regression tests
use relative tolerance 3e-6 and absolute tolerance 2e-7 for matrix comparisons.

For a single RR movement, only column 3 of the raw matrix should change: its roll
and pitch entries are flattened indices 3 and 7. Yaw index 11 and vertical-thrust
index 23 stay constant if coefficients are unchanged. Returning to neutral must
restore the neutral matrix with unchanged coefficients and CoG.

The assigned matrix should equal the raw matrix except that handled failed-motor
columns are zero and rows whose entries all have absolute value <=1e-6 are zero.
The record is the assigned **B**, not its pseudoinverse or the normalized mixing
matrix; do not directly multiply it by normalized motor logs to claim an exact
physical wrench without accounting for allocator normalization.

These logs establish input-to-geometry-to-assignment consistency. They do not
measure servo position or prove that a physical arm reached its commanded angle.

Diagnostics tests verify matrix flattening, coefficient/position consistency,
assignment after simulated masking, timestamp/generation matching, rejected and
unchanged inputs, and NaN diagnostics for invalid geometry configuration.

## ARM stack regression check

The first ARM image exposed excessive nested stack use when selecting
CA_AIRFRAME=13: the update function's 1,984-byte frame remained live during
geometry diagnostics and normalization. The known call paths alone exceeded the
3,150-byte rate-control work-queue stack (3,688 bytes for geometry logging and
4,136 bytes for normalization). This is a concrete defect that can cause FC faults;
it is not hardware confirmation of any particular USB disconnect.

Large configuration, normalization and actuator-limit buffers now live in the
ControlAllocator object. They are value-initialized in place on each update to
avoid both stale data and a hidden stack temporary; this performs no per-update
heap allocation. GCC 9-2020-q2 reports a 136-byte update frame and checked path
sizes of 1,848 and 2,296 bytes respectively. The additional object storage uses
about 1.8 KiB of heap for the buffers, not an enlarged shared work-queue stack.

Run the check against the actual configured ARM build (not the desktop build):

```sh
python3 Tools/morphing_quad_geometry/check_arm_stack.py --build-dir /path/to/arm-build
```

It recompiles four translation units into a temporary directory with GCC's
`-fstack-usage`, checks the large frame and known nested paths, and reserves at
least 512 bytes of headroom. This is a static regression check, not a full call-
graph proof or measured hardware stack high-water mark. The new
`functional-ControlAllocatorMorphing` host test repeatedly switches 0 -> 13 -> 0,
reconfigures the actual ControlAllocator and verifies buffer reset and assigned
matrix publication.

## Console-only angle test

With the vehicle disarmed, propellers removed, and `CA_AIRFRAME=13`, run:

```sh
control_allocator morphing_test 0 0 0 0
control_allocator status
control_allocator morphing_test 10 0 0 0
listener morphing_arm_state -n 1
listener morphing_allocation_matrix -n 1
control_allocator status
control_allocator morphing_test 0 0 0 0
```

Arguments are FR, RR, RL, FL commanded angles in degrees, each within +/-30.
The test converts them to radians and publishes `morphing_arm_state` once.
The publisher is owned by the running allocator, so its advertisement remains
available for a listener invoked after the command. A command-local static C++
publisher is insufficient on NuttX because its destructor may run at task exit.
No PWM or servo position command is sent. The allocator does consume these
simulated angles: restore all four to zero when finished, and do not arm or fly
with simulated geometry. Do not use this test alongside a real angle publisher.
The command rejects missing/stale vehicle status, armed vehicles, other airframe
selections, malformed numbers, and out-of-range angles.

For a front-right-only angle change, only motor 0's roll/pitch effectiveness
should change; yaw and thrust coefficients remain unchanged. A message alone
proves publication, while `morphing_allocation_matrix.timestamp_angle` matching
its timestamp confirms that the allocator applied that angle sample.

## FC bench result (2026-09-10)

User-provided NxtPX4v2 console output confirmed source 13 persisted after reboot,
the angle listener printed `[0.17453, 0, 0, 0]`, and a front-right-only 10 degree
command changed motor 0 roll/pitch entries from `-0.44094 / 0.44152` to
`-0.39194 / 0.48254`. Other motor entries remained unchanged. Publishing four
zeros restored the neutral matrix and the listener displayed zero angles.
This validates a simulated command through uORB to the allocator on the FC;
no physical servo movement or flight was tested.

## Numerical inverse scaling

The morphing provider opts into numerical row scaling for both the actual matrix
and its neutral normalization reference. The four active rows (roll, pitch, yaw,
vertical thrust) are divided by their respective largest absolute entries before
calling the existing float `matrix::geninv` on a compact 4x4 matrix. Its inverse
is then converted back to physical units before the existing PX4 normalization.
This prevents the much larger thrust coefficients from degrading the small
physical torque coefficients during inversion. No geometry, parameter, motor
ordering, update-rate, desaturation, clipping, or servo logic is changed.

Scaling is only accepted for a finite, four-motor matrix with zero lateral thrust
and a full-rank active map, verified by the scaled inverse residual. Failed-motor,
rank-deficient, unsupported, or numerically rejected cases use the original
unscaled pseudoinverse. This preserves the original least-squares weighting in
those cases. Other effectiveness providers do not enable this policy.

The implementation uses fixed-size float matrices and adds no allocation in the
update loop. The ARM stack check includes both the scaled and fallback paths
during normalization and ordinary allocation. See [inverse validation results](INVERSE_VALIDATION.md)
for the reproduced error, regression coverage, and target build measurements.

## Direction-preserving saturation

The morphing provider also opts into bounded roll/pitch-preserving saturation
with Auto or sequential allocation. It corrects the folded-geometry torque
reversal found after the inverse change. See [saturation validation](SATURATION_VALIDATION.md)
for the priority policy, supported configurations, regressions, and hardware limits.
