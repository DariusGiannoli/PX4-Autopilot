# Morphing inverse validation — 2026-09-16

This change corrects the reproduced float inversion error for the morphing quad
(`CA_AIRFRAME=13`). It leaves the physical effectiveness matrix and the existing
control-allocation workflow intact.

## Calculation

For the full-rank 4x4 active matrix A, let S be diagonal, with each diagonal entry
equal to the reciprocal of that row's largest absolute coefficient. Calculate
`As = S*A`, then `A_inverse = geninv(As)*S`. Map the result back to the existing
16x6 mixer, leaving lateral-thrust and unused-motor entries zero. Neutral-reference
normalization then proceeds in the original physical units.

The same helper computes the neutral reference and each updated inverse. The
compact matrices keep stack usage below a second full 6x16 temporary. Everything
uses float and the existing PX4 matrix library. There is no new heap allocation
or external numerical dependency.

An identity-residual check (maximum entry error 1e-4) prevents acceptance of a
rank-deficient pseudoinverse as a full inverse. Motor failures, absent axes,
unsupported matrix shapes, and rejected numerical results retain the original
unscaled solver. Row scaling in a rank-deficient problem would change its
least-squares weights, so it is deliberately excluded. Standard PX4 providers
retain the original path by default.

## Regression results

The new test was run before the fix and failed, then passed after the fix. Each
profile covers all 81 combinations of -30/0/+30 degrees and 5,000 seeded random
geometries. For each geometry, a known feasible motor vector u in [0.2, 0.7]
generates the requested wrench through B*u and neutral normalization. Allocation
must recover u within 1e-4. This oracle does not compute a second pseudoinverse.

| Coefficients | Largest motor error before | Largest motor error after |
| --- | ---: | ---: |
| CT=1, balanced KM=+/-0.006 | 0.171415 | 3.58e-7 |
| CT=6.5, balanced KM=+/-0.05 | 0.000622481 | 4.18e-7 |
| Unequal CT=[0.8,1.1,1.2,0.9], balanced KM=+/-0.006, CoG=[0.009,-0.007,0] m | 0.894077 | 6.56e-7 |

These are normalized motor-command errors on the host, not measured motor thrust
or measurements from the FC. Each of the 15,243 geometries also receives an
infeasible command; outputs remain finite and within limits after desaturation
and clipping. Neutral normalization remains fixed during geometry updates.

All 29 tests in five targeted suites pass: morphing provider (8), actual allocator
module (3), pseudoinverse (5), sequential desaturation (7), and arm publisher (6).
The module test verifies policy wiring for both allocation methods. Unit tests
verify physical inverse units, analytically known neutral scaling, policy cache
invalidation, and legacy results for default/disabled policy and 12 fallback
cases, including each missing motor, dependent rows/columns, and zero matrices.

The host tests recompiled all 21 allocator/effectiveness/publisher translation
units and the five test sources from this checkout, linking the existing desktop
support libraries. Separately, 1,081 geometry comparisons against the CAD model
pass under address and undefined-behavior sanitizers; maximum matrix-entry error
is 2.42e-7. Geometry code itself is unchanged.

## Target build and stack

A complete `hkust_nxt-dual_default` build succeeds with GCC ARM 9.3.1, CMake 3.31.10
and the board's default configuration. A separate space-free source copy was
used because the NuttX build contains shell paths that do not tolerate spaces.

- Firmware image: **1,822,216 / 1,835,008 bytes**, leaving **12,792 bytes**.
- Linked static AXI SRAM usage: **69,580 / 524,288 bytes**; this excludes runtime
  heap and stack consumption.
- No firmware was flashed and no FC parameters were changed.

The expanded GCC `-fstack-usage` check reports:

| Known nested path | Bytes |
| --- | ---: |
| Geometry logging | 1,848 |
| Neutral normalization, scaled inverse | 1,304 |
| Neutral normalization, legacy fallback | 2,312 |
| Allocation, scaled inverse | 1,384 |
| Allocation, legacy fallback | 2,392 |

All checked paths fit the 3,150-byte rate-control stack with the check's additional
512-byte reserve. The update function still has a 136-byte frame. This is a static
regression check, not a complete call-graph proof or an on-hardware high-water
measurement. FC execution time and flight behavior have not been measured.

The existing [test guide](README.md) gives the host-test commands. Run the updated
stack script against the actual configured ARM build directory:

```sh
python3 Tools/morphing_quad_geometry/check_arm_stack.py --build-dir /path/to/arm-build
```
