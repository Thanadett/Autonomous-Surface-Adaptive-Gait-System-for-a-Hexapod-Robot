# E1 (Gazebo, body fixed in the air) - foot-tip error of IK + controller

- Generated 2026-09-25 16:38:46; IK solver: analytic; move 0.6 s + settle 1.2 s, joint positions averaged over the last 0.2 s (sim time); wall time 9 s
- Targets: E1 walking grid, 5 per leg x 6 legs = 30 (see hexapod_evaluation/e1_targets.py)
- Foot-tip error = |FK(q_measured) - target|

**All legs: 0.01 ± 0.01 mm (mean ± SD), p95 0.02 mm, max 0.02 mm, n = 30**

| Leg | n | Mean ± SD (mm) | p95 (mm) | Max (mm) |
|---|---:|---:|---:|---:|
| front_left | 5 | 0.01 ± 0.01 | 0.02 | 0.02 |
| middle_left | 5 | 0.01 ± 0.01 | 0.02 | 0.02 |
| rear_left | 5 | 0.01 ± 0.01 | 0.02 | 0.02 |
| front_right | 5 | 0.01 ± 0.01 | 0.02 | 0.02 |
| middle_right | 5 | 0.01 ± 0.01 | 0.02 | 0.02 |
| rear_right | 5 | 0.01 ± 0.01 | 0.02 | 0.02 |

| Axis (body frame) | Mean abs error ± SD (mm) | Max (mm) |
|---|---:|---:|
| x | 0.00 ± 0.01 | 0.02 |
| y | 0.01 ± 0.00 | 0.02 |
| z | 0.00 ± 0.01 | 0.02 |

| Joint | Mean abs(q_meas - q_cmd) ± SD (deg) | Max (deg) |
|---|---:|---:|
| coxa | 0.002 ± 0.002 | 0.008 |
| femur | 0.003 ± 0.004 | 0.015 |
| tibia | 0.004 ± 0.003 | 0.016 |

IK residual |FK(q_cmd) - target|: mean 0.000000 mm, max 0.000000 mm; IK failures: 0
