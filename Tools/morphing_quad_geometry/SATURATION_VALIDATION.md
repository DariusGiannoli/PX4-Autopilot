# Morphing saturation correction — 2026-09-17

Branch `codex/morphing-saturation` corrects the reproduced roll/pitch reversal
under saturation. The [baseline investigation](ROLL_PITCH_VALIDATION.md) records
the failure on firmware `c6f77d6a3557`. This is an allocator correction validated
on the host and compiled for HKUST_NXT_DUAL; it is not flight validation.

## Behavior and scope

The morphing provider opts into a bounded four-motor calculation in the sequential
allocator. It is selected with `CA_AIRFRAME=13` and `CA_METHOD=2` (Auto, as in the
recorded setup) or `CA_METHOD=1`. Other providers default to the existing behavior.
Explicit pseudoinverse-only allocation (`CA_METHOD=0`) keeps its existing clipping
behavior and does not receive this saturation correction.

For a supported full-rank vertical four-motor map, feasible commands retain the
inverse solution, subject to float roundoff. When a command exceeds motor limits:

1. Temporarily remove yaw. Find the largest common roll/pitch multiplier in
   `[0,1]` that fits the motor bounds and allowed collective. Keeping a common
   multiplier preserves the requested roll/pitch direction and their ratio.
2. At that multiplier, choose collective nearest the requested collective.
3. Retain that roll/pitch solution and add as much of the requested yaw as fits,
   preferring the collective chosen in step 2.

This implements a specific two-stage priority policy. It is not a global weighted
optimization of all four control demands. Yaw can be sacrificed to retain roll
and pitch. Collective can decrease when the requested combination is infeasible.
The existing airmode meanings are respected: mode 0 permits no collective increase
above the request; mode 1 permits an increase for roll/pitch but not for adding yaw;
mode 2 also permits collective adjustment while adding yaw. At zero collective in
mode 0, torque authority is zero.

Each stage evaluates 25 fixed inequalities after eliminating collective from
`0 <= base[i] + direction[i]*scale + thrust[i]*collective <= motor_max[i]`.
There are no iterations, heap allocations, additional inversions, or external
dependencies. A small float margin keeps the solution inside active boundaries.

Guards require four non-reversible motors with zero minimum and trim, finite
coefficients, a valid normalized inverse, positive pure-collective motor
coefficients, no lateral thrust demand/effectiveness, and no actuator failure.
Unsupported shapes, rank loss, and failed-motor configurations retain the existing
allocator. This change does not establish failed-motor flight capability. Numerical
rejection within the supported calculation retains feasible zero-torque collective
or the accepted roll/pitch solution with yaw removed.

The geometry, servo calibration/zero mapping, neutral normalization, inverse
calculation, rate/attitude controllers, and output mapping are unchanged. No new
parameters are needed. The checks apply at the allocator output; downstream motor
slew limiting and physical motor/servo dynamics are separate effects.

## Reproduced case, now corrected

All arms +30 degrees, CT=6.5, KM in motor order FR/RL/FL/RR =
`[-0.05,-0.05,+0.05,+0.05]`, fixed CoG at CAD origin, `MC_AIRMODE=0`:

| Normalized quantity | Requested | Before | After |
| --- | ---: | ---: | ---: |
| Roll | 0 | +0.440180 | approximately 0 |
| Pitch | +1 | -0.242114 | +0.271950 |
| Yaw | 0 | -0.070664 | approximately 0 |
| Fz | -0.25 | -0.360552 | -0.250000 |

New motor commands in FR/RL/FL/RR order are approximately
`[0.500000, 0.000000, 0.378904, 0.121096]`. The full pitch request is infeasible;
the correction produces the attainable positive pitch under this policy. These
are normalized software quantities, not stick positions, angles, or measured thrust.

## Regression evidence

All **35 tests in five targeted suites pass**: morphing provider 13, actual
allocator module 4, pseudoinverse 5, sequential desaturation 7, arm publisher 6.
The original failing assertions remain enabled and now pass.

| Coverage | Result |
| --- | --- |
| 5,616 feasible cases across 144 geometry evaluations | maximum normalized tracking error 1.90e-7 |
| 1,728 saturated directional probes, including recovery | no requested roll/pitch sign reversals |
| 52,128 saturation property cases | finite bounded outputs, preserved roll/pitch direction, zero-axis and airmode checks pass |
| 8,748 independent double-precision comparisons | maximum motor-command difference 4.27e-6 |
| Nine unsupported/failure configurations | results match the original allocator |
| Actual module with Auto and explicit sequential selection | corrected all-arms +30 degree case passes |
| Existing inverse recovery across 15,243 geometries | regression remains passing |

Property coverage includes all three airmodes, 81 corner geometries plus seeded
random geometries, zero/low/high collective, mixed torque demands, unequal CT and
CoG offsets. Maximum roll/pitch direction error is 3.47e-7; maximum unintended
torque on an axis commanded zero is 2.98e-7. The independent oracle inverts the
normalized matrix in double precision and enumerates intersections of constraint
boundaries; production uses float interval elimination. It checks the two-stage
policy, not an alternative global objective.

Geometry transitions are evaluated offline. These tests do not simulate servo
lag, motor dynamics, aerodynamic interference, moving inertia, vibration, or the
closed-loop aircraft. The host runner's work-queue-unavailable startup messages
are expected in this test environment; the suites do not run on the FC.

Reproduce from a configured host build:

```sh
cmake --build build/morphing-tests --target \
  functional-ActuatorEffectivenessMorphingQuad functional-ControlAllocatorMorphing \
  unit-ControlAllocationPseudoInverse functional-ControlAllocationSequentialDesaturation \
  functional-MorphingArmPublisher
ctest --test-dir build/morphing-tests --output-on-failure --timeout 30 \
  -R '^(functional-ControlAllocatorMorphing|functional-ActuatorEffectivenessMorphingQuad|unit-ControlAllocationPseudoInverse|functional-ControlAllocationSequentialDesaturation|functional-MorphingArmPublisher)$'
```

## Board build, stack, and next verification

The `hkust_nxt-dual_default` ARM build succeeds with GCC 9.3.1. The generated
`build/hkust_nxt-dual_saturation/build-manifest.json` identifies the exact source
commit, image size, checksums, and saved evidence. Artifacts are generated files,
not tracked firmware binaries. The separate space-free ARM source checkout is
needed by the NuttX build scripts on this machine.

The updated GCC static stack audit includes the new helper's nested frame. Its
known path uses **488 bytes**. The largest audited path remains the legacy inverse
fallback at **2,392 bytes**, within the rate-control queue's **3,150 bytes** with
the audit's **512-byte reserve**. The matrix-update frame remains 136 bytes.
This is a regression budget check of known frames, not a complete runtime stack
proof. FC execution time and stack high-water usage still require measurement.

```sh
python3 Tools/morphing_quad_geometry/check_arm_stack.py --build-dir /path/to/arm-build
```

No firmware was flashed, no FC parameters changed, and no hardware was operated
for this correction. After a separately authorized flash, repeat props-off startup,
parameter/calibration retention, neutral geometry, motor identity/direction, and
the logged arm-motion checks. Check rate-control/allocator timing and runtime stack
headroom on the FC. Direct actuator testing bypasses allocation and cannot validate
the torque correction itself. Do not attempt the synthetic +1 pitch regression
with propellers installed.

The reported 13–16 A with stopped motors and unresolved preflight checks remain
separate issues to investigate. Passing this correction's tests does not establish
that the complete firmware or aircraft is ready for flight or in-flight morphing.
