# Morphing firmware verification — 2026-10-05

## Result and scope

The allocation geometry agrees with the independent CAD dynamics model, and all
35 tests in the five targeted host suites pass after a fresh compilation. No new
allocation implementation defect was found in this review. This establishes
software consistency for the tested configurations; it does not establish
closed-loop flight stability or correct physical output wiring.

Reviewed firmware: `codex/morphing-saturation`, commit
`0b0ba519bcc0bb2b6480ab8ab66282f7cb2a267b`. The runtime sources were clean and
were not changed by this audit. Verification tooling now detects the renamed
sibling `Morpheus` repository, retaining the former repository name as a fallback.
The geometry README was corrected to describe the current inverse and saturation
policies. No FC was connected, flashed, configured or operated during this audit.

## Allocation mathematics checked

Use PX4 body forward/right/down coordinates, with wrench rows
`[tau_x, tau_y, tau_z, F_x, F_y, F_z]`. Input arm angles are radians, in
FR/RR/RL/FL order, limited to ±30 degrees. Positive CAD joint rotation maps to
negative rotation about PX4 body Z:

```text
r_i(alpha_i) = h_i + R_z(-alpha_i) d_i
l_i = r_i - c
B_i = CT_i [-l_iy, l_ix, KM_i, 0, 0, -1]^T
w = B(alpha) u
```

Here `h_i` is the hinge position, `d_i` the neutral hinge-to-thrust-site offset,
`c` the configured CoG, and `u` the four motor effectiveness inputs. `KM_i` uses
the signed PX4 reaction-torque convention. Matrix columns and motor outputs use
FR/RL/FL/RR order; coefficient parameters use that same motor order. The provider
correctly remaps coefficients to input arm order before calculating geometry.

The reference comparison checks the CAD-to-PX4 rotation
`(x,y,z)_PX4 = (-y,-x,-z)_CAD`, angle signs, lever arms, motor ordering, reaction
torque signs and translation of torque to the supplied CoG. `CT=1` with physical
thrust inputs gives a physical wrench map. In firmware, CT scales effectiveness,
and PX4 additionally normalizes the inverse: controller setpoints must not be
interpreted as SI wrench values without accounting for that normalization.

The active rows `[tau_x,tau_y,tau_z,F_z]` form a four-by-four map `A`. The guarded
row-scaled inverse computes `A^-1 = (S A)^-1 S`, verifies the identity residual,
and falls back to the existing PX4 pseudoinverse for unsupported or deficient
maps. Normalization uses neutral geometry and remains fixed while the arms move;
the current geometry is still inverted. This avoids changing controller scaling
on every arm movement.

With Auto/sequential allocation, the guarded saturation calculation first keeps
the largest feasible common roll/pitch multiplier, then adds feasible yaw. Tests
include the formerly failing all-arms +30-degree pitch case. Pseudoinverse-only
allocation (`CA_METHOD=0`) does not receive this saturation correction. Failed
motor cases fall back to existing allocation; this is not verification of
motor-out flight capability.

## Fresh host verification

| Check | Result |
| --- | --- |
| Actual C++ geometry header versus CAD/URDF model | 1,081 configurations passed: 81 neutral/limit combinations plus 1,000 seeded random configurations |
| Maximum rotor-position difference | 9.09e-9 m |
| Maximum matrix-entry difference | 2.42e-7, with the tested coefficient profiles |
| Frame/sign/order, fixed and changing reference CoG, unequal CT, composed wrench and rank | Passed |
| Invalid input, overflow, unchanged output on rejection | Passed |
| Clang C++14 warnings as errors, address and undefined-behaviour sanitizers | Passed for the geometry probe |
| Actual allocator-module integration | 4 tests passed |
| Morphing effectiveness provider, inverse recovery, saturation properties and independent vertex oracle | 13 tests passed |
| Pseudoinverse regression | 5 tests passed |
| Sequential-desaturation regression | 7 tests passed |
| Calibrated arm publisher and command-to-matrix integration | 6 tests passed |

Reference files in `Morpheus` were read without modification:

- URDF SHA256: `b1f12853f90f91386f4f2aa743f5d4d0eff1900991357bec839c792d968655c6`
- Dynamics model SHA256: `2e41a54813633bca8298d08204c49d21f6f83b29c84d042195f96029fb9105fc`

The new host build is `build/morphing-audit-host`; logs and the isolated Python
environment are in `build/morphing-audit-tools`. CMake 3.31.10 is used because
the older PX4 dependencies are incompatible with the installed CMake 4 defaults.
The host configuration suppresses Clang's new VLA-extension-as-error diagnostic;
it does not alter firmware compiler flags. Work-queue-unavailable and rejected
configuration messages are expected in these manually driven test fixtures:
these tests do not execute the FC scheduler. Sanitizers above apply to the
separate geometry probe, not the complete firmware or all five suites.

Reproduce using a compatible PX4 test build and Python environment:

```sh
python3 Tools/morphing_quad_geometry/test_geometry.py
cmake --build build/morphing-audit-host --target \
  functional-ControlAllocatorMorphing functional-ActuatorEffectivenessMorphingQuad \
  unit-ControlAllocationPseudoInverse functional-ControlAllocationSequentialDesaturation \
  functional-MorphingArmPublisher modules__control_allocator modules__morphing_arm_publisher
ctest --test-dir build/morphing-audit-host --output-on-failure --timeout 30 \
  -R '^(functional-ControlAllocatorMorphing|functional-ActuatorEffectivenessMorphingQuad|unit-ControlAllocationPseudoInverse|functional-ControlAllocationSequentialDesaturation|functional-MorphingArmPublisher)$'
```

## Runtime integration findings

The command-to-geometry path is:

```text
MAVLink DO_SET_ACTUATOR (group 0, four normalized targets)
  -> vehicle_command
     -> FunctionActuatorSet -> configured peripheral PWM outputs -> servos
     -> morphing_arm_publisher -> calibrated target angles -> morphing_arm_state
        -> ActuatorEffectivenessMorphingQuad -> B(alpha)
           -> inverse, neutral normalization and saturation
           -> actuator_motors -> configured PWM/DShot outputs -> ESCs -> motors
```

The classical rate controller supplies torque and collective demands to the
allocator. The arm publisher drains commands at a nominal 100 Hz on `lp_default`.
Ordinary changed-geometry rebuilds are limited to 100 Hz inside allocation runs
on `rate_ctrl`; this is not the motor output rate. Publisher diagnostics run at
10 Hz. Actual rate-controller, sensor, PWM/DShot and estimator rates depend on
configuration and require FC measurements.

Relevant constraints before flight:

1. **Geometry is based on targets, not measured angles.** Servo motion is assumed
   immediate. Fixed-arm tests should use measured, settled configurations.
   Moving-arm flight requires characterization of servo lag, tracking and the
   resulting geometry mismatch. No moving-inertia or dynamic-CoG estimator is
   implemented; `CA_MORPH_COGX/Y/Z` is a fixed configured offset.
2. **Startup and stale state do not enforce an arming interlock.** Before an
   accepted state the provider uses neutral geometry. The publisher rejects
   uncalibrated startup and diagnoses expired commands, but the allocator holds
   its last accepted geometry if publication stops. A successful diagnostic
   message does not confirm that PWM was enabled or a servo reached its target.
3. **Servo output and angle publication are independent consumers.** RC AUX,
   actuator tests, overrides and safety/failsafe output changes are not observed
   by the publisher. Use its documented command path and verify actual output
   mapping; malformed command fields can be treated differently by the two
   consumers.
4. **The board does not activate morphing by default.** The compiled defaults
   retain `CA_AIRFRAME=0` and `MORPH_PUB_EN=0`. Verify saved `CA_AIRFRAME=13`,
   `CA_ROTOR_COUNT=4`, `CA_METHOD=2` or `1`, motor CT/KM, fixed CoG, peripheral
   assignments and measured MARM calibration on the actual FC. Calibration and
   timeout changes require restarting the publisher.
5. **External MoCap reception is not automatically EKF2 fusion.** In this source,
   MAVLink ODOMETRY tagged `MAV_ESTIMATOR_TYPE_MOCAP` is routed to
   `vehicle_mocap_odometry`; EKF2 subscribes to `vehicle_visual_odometry`.
   Validate the bridge's external-vision route, frame transforms, timestamps,
   covariance and EKF2 fusion settings. This audit did not run a MoCap stream or
   change the estimator route.

## Next hardware verification

With the FC available, begin with a disarmed, propellers-removed session. Record
the installed firmware revision and export parameters before deciding whether
the newly compiled image needs flashing. Check startup/hardfault logs, output
identity, motor direction, measured servo calibration and sensor/estimator health.

Log `morphing_publisher_status`, `morphing_arm_state`, `morphing_geometry`,
`morphing_allocation_matrix`, allocation status and actuator outputs. At each
settled neutral/asymmetric/limit configuration, compare measured arm angles and
rotor positions to the diagnostics and compare the applied matrix to the
reference. Check repeated commands, timeout and reinitialization. Measure
allocator runtime, work-queue stack high-water usage and output timing.

Direct actuator tests bypass allocation and cannot establish correct torque
allocation. The synthetic console angle command does not move servos and must
not be mistaken for an estimate of the physical configuration. The next flight
milestone is a classical fixed-arm hover after those checks and resolution of
preflight/power anomalies; moving-arm and RL flight require their own validation.

## ARM build and static stack evidence

The full `hkust_nxt-dual_default` build passed with GNU Arm Embedded GCC 9.3.1,
CMake 3.31.10 and the pinned NuttX/submodule revisions. It includes EKF2,
conventional multirotor controllers, control allocation, the arm publisher,
MAVLink, PWM and DShot. A fresh detached source checkout outside the workspace
was used because the NuttX scripts require a source path without spaces. Its
runtime sources match the reviewed commit; no old firmware image was reused.
The optional SymForce regeneration dependency is absent; the configuration uses
the checked-in generated estimator sources, and the complete build succeeds.

| Resource | Fresh result |
| --- | --- |
| Board ID | 1013 |
| Firmware image | 1,824,040 bytes of 1,835,008 bytes |
| Flash space remaining | 10,968 bytes (about 0.6%) |
| Static AXI SRAM use | 69,580 bytes of 524,288 bytes; excludes dynamic allocations and runtime stack use |
| Matrix-update function stack frame | 136 bytes |
| Largest audited nested path | 2,392 bytes, legacy inverse fallback |
| Direction-preserving saturation path | 488 bytes |
| Rate-control work-queue stack | 3,150 bytes; audited maximum plus 512-byte reserve fits |

The static stack regression check passed. It sums selected known GCC-reported
frames and reserves space for uncounted calls; it does not prove every runtime
call path, scheduler overhead or interrupt behavior. Measure actual high-water
usage and allocator execution time on the FC. Flash space is tight, so future
firmware additions may require trimming unused modules.

Artifacts, checksums, source hashes and build/test logs are saved in
`build/hkust_nxt-dual_audit_2026-10-05/build-manifest.json` and its directory.
The original older build evidence is preserved. This fresh image has not been
flashed or executed on hardware.
