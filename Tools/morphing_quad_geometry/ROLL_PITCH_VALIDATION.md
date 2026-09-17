# Roll/pitch allocation validation — 2026-09-17

> Historical baseline report for firmware `c6f77d6a3557`. The failure below was
> subsequently corrected on `codex/morphing-saturation`; both failing tests now
> pass. See [the saturation validation report](SATURATION_VALIDATION.md) for the
> current results, scope, and remaining hardware validation. The following text
> preserves the original investigation and its pre-fix measurements.

## Decision

**FAIL: saturated allocation can reverse requested torque in folded configurations. Postpone flight testing until this behavior is corrected and retested.**

The ordinary command-to-geometry pipeline and feasible inverse allocation pass. The new tests expose a separate problem in the sequential-desaturation path used with the recorded MC_AIRMODE=0 configuration. This is a reproducible host-software result, not an observed in-flight event.

Only test code and this report were added on `codex/stabilize-morphing-allocation`. Production firmware code, hardware, parameters, and the arm-control script were not changed by this task. No new firmware needs flashing to run these host tests.

## Configuration and coverage

The setup matches the latest log: CA_AIRFRAME=13; CA_METHOD=2 (Auto selects sequential desaturation for this provider); MC_AIRMODE=0; CT=6.5 for all motors; motor-order KM=[-0.05,-0.05,+0.05,+0.05]; fixed CoG at the CAD origin; neutral normalization; motor bounds 0..1.

The real C++ geometry provider, normalization, pseudoinverse, sequential desaturation, and final output clipping were compiled and exercised on the host.

- All 81 combinations of -30/0/+30 degrees in FR/RR/RL/FL arm order.
- 63 additional geometry evaluations: 0→30→0 in 3-degree steps for FR alone, the FR/RL diagonal, and all four arms.
- The requested neutral, FR-only +30, FR/RL +30, and all-arms +30 poses are included.
- 13 feasible demands per geometry at normalized collective 0.25, 0.40, and 0.50: zero torque, positive/negative roll and pitch at 0.05 and 0.15, and four combined ±0.05 roll/pitch demands.
- Saturation probes: positive/negative roll or pitch demand 1.0 at the same three collective levels; finite/range checks, torque-direction checks, and recovery to a feasible neutral-torque command.

These are normalized allocator quantities, not stick positions, aircraft angles, measured thrust, or recommended flight settings. Geometry transitions are evaluated sequentially offline; no real-time control or servo dynamics are simulated.

## Passing results

All **5,616 feasible allocation cases pass** across 144 geometry evaluations.

- Maximum normalized forward-control error: **1.89990e-7**.
- Maximum unintended torque on an axis commanded zero: **9.6775e-8**.
- Active motor commands stay between **0.052794 and 0.783923**, before clipping.
- Normalization stays fixed to neutral through geometry changes.
- An analytical neutral normalization check passes.
- All 1,728 saturation probes remain finite and bounded after clipping, and recover correctly when returned to a feasible neutral-torque demand.

An independent double-precision solve confirmed that the selected feasible command set stays within motor limits for the corner configurations. A higher exploratory collective of 0.75 did not meet that condition for all asymmetric poses and was therefore not classified as a universally feasible test point.

## Confirmed failure

**56 of 1,728 saturation probes produce torque opposite to the requested roll/pitch direction.** This count includes repeated poses encountered during the transition sweeps; it is not a statistical flight-failure probability. The remaining probes passing the sign check does not mean all their cross-axis or thrust behavior is acceptable.

Minimal example, starting with a fresh allocator:

| Quantity | Requested | Allocated |
|---|---:|---:|
| Roll | 0 | +0.440180 |
| Pitch | +1.000000 | **-0.242114** |
| Yaw | 0 | -0.070664 |
| Vertical thrust | -0.250000 | -0.360552 |

All four arms are +30 degrees. Final motor commands, ordered FR/RL/FL/RR, are:

```text
[0.000000, 0.862432, 0.579777, 0.000000]
```

The requested +1 pitch is infeasible at this collective; its unconstrained motor solution contains negative values and a value above one. Exact tracking is therefore not expected. **That does not require reversing pitch or introducing large roll torque.** A feasible witness at the same collective is:

```text
motors = [0.433857, 0.066143, 0.344800, 0.155200]
result = [roll 0, pitch +0.200000, yaw 0, vertical thrust -0.250000]
```

These forward results were independently verified using double-precision CAD-derived geometry, without the allocator's `getAllocatedControl()` helper.

The minimal reproduction compares fresh allocators with the row-scaled inverse enabled and disabled:

| Pose | Pitch request | Updated inverse: achieved pitch | Original inverse: achieved pitch |
|---|---:|---:|---:|
| All 0 degrees | +0.15 | +0.150000 | +0.150000 |
| All 0 degrees | +1.00 | +0.352697 | +0.352697 |
| All +15 degrees | +0.15 | +0.150000 | +0.150000 |
| All +15 degrees | +1.00 | +0.060532 | +0.060532 |
| All +30 degrees | +0.15 | +0.150000 | +0.150000 |
| All +30 degrees | +1.00 | **-0.242114** | **-0.242115** |

At +15 degrees, the saturated +1 pitch case also creates approximately +0.263 unintended roll. Both inverse policies behave essentially identically. This evidence points to the existing sequential-desaturation handling for the folded geometry, rather than the inverse-accuracy change. No corrective algorithm change was attempted in this testing task.

The current saturation algorithm makes sequential adjustments along thrust, roll, pitch and yaw mixing vectors. Bounds alone are insufficient: these tests show why achieved torque direction and cross-axis behavior also need checking.

## Test status and reproduction

Added to `src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMorphingQuadTest.cpp`:

- `RecordedConfigurationRollPitchDemandsAndTransitions`: feasible checks pass; 56 saturated torque-direction assertions fail.
- `FoldedSaturatedPitchMustNotReverseDirection`: minimal fresh-allocator comparison; two assertions fail, one for each inverse policy at +30 degrees and pitch +1.

The five targeted regression executables were rebuilt. The original **29 tests pass**; the **two newly added tests fail** on the behavior described above. They are intentionally left enabled and failing so a future fix must address the problem rather than merely relax the assertions.

From a configured PX4 host-test build:

```sh
cmake --build build/morphing-tests --target functional-ActuatorEffectivenessMorphingQuad
build/morphing-tests/functional-ActuatorEffectivenessMorphingQuad \
  --gtest_filter='MorphingAllocationTest.RecordedConfigurationRollPitchDemandsAndTransitions:MorphingAllocationTest.FoldedSaturatedPitchMustNotReverseDirection' \
  --gtest_output=xml:build/morphing-tests/roll-pitch-validation.xml
```

The existing desktop build cache referenced a stale temporary CMake/Python installation. It was reconfigured with the available CMake 3.31.10 and the existing build virtual environment. This changed generated build files only. The host functional runner emits the same work-queue-unavailable startup diagnostics as the prior functional tests; no live FC or work-queue hardware execution is involved.

Saved evidence is under `build/morphing-tests/roll-pitch-evidence/` (generated, untracked build artifacts): build log, full five-suite test log, detailed saturation log, minimal reproduction, XML results and independent forward calculation.

## What to do on the drone now

1. **Postpone flight and in-flight morphing tests.** Correct and retest saturation handling first. Do not attempt to reproduce the +1 pitch case on the real aircraft, and do not change airmode or disable checks to hide the failure.
2. With **all propellers removed**, secure the aircraft and use QGC's **Vehicle Setup → Actuators → Actuator Testing** to identify one logical motor at a time at the lowest output that reliably spins it. Return its slider to zero before testing the next. Expected mapping from the recorded configuration:

   | QGC logical motor | Physical location | MAIN output |
   |---|---|---:|
   | Motor 1 | Front-right | 1 |
   | Motor 2 | Rear-left | 4 |
   | Motor 3 | Front-left | 3 |
   | Motor 4 | Rear-right | 2 |

   This direct test checks wiring/order and observed spin direction. It bypasses allocation and is **not** a roll/pitch control test. Do not use actuator-test servo sliders to validate the calibrated morphing publisher. See the official [PX4 actuator-testing instructions](https://docs.px4.io/main/en/config/actuators).
3. With the aircraft disarmed, check that QGC's attitude display agrees with physical forward direction, nose-up/down, and left/right tilt. This checks attitude orientation, not motor correction or flight stability.
4. Record any wrong motor location or reversed attitude indication. The approximately 13–16 A reported bench current still needs calibration/load investigation before flight.

The next engineering step is a correction to saturated allocation that preserves sensible requested torque direction, controls unintended torque/thrust, and passes these regressions. Successful geometry logging by itself does not cover this failure mode.
