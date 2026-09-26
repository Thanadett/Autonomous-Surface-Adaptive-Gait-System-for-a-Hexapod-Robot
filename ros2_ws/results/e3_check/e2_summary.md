# E3 - gait x surface (chapter 4.1.3, Gait Performance Map)

- Generated 2026-09-26 10:23:49; seed 2; course 0.3 m, success = goal within 2.0 x nominal time without a fall (|roll|/|pitch| > 30.0°, z < 60 mm)
- Mass 4.147 kg; cycle times {'tripod': 0.95, 'ripple': 1.3, 'wave': 2.4}; metrics exclude the warm-up (1 gait cycle); Mean ± SD (sample SD) over all trials of a condition
- Ground truth only (OdometryPublisher, joint states, contact and joint F/T sensors); slip = sliding of the foot contact point per stance, centre slip = ball-centre displacement

| Condition | Cmd (mm/s) | Success | Falls | Speed (mm/s) | Speed / cmd (%) | SSM min (mm) | SSM p5 (mm) | SSM mean (mm) | SSM min, contact only (mm) | Support feet | Feet in contact | Slip/stance (mm) | Centre slip (mm) | CoT | Roll RMS (°) | Pitch RMS (°) | Lateral drift (mm) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| tripod_mu100 | 25 | 1/1 (100%) | 0 | 22.4 ± 0.0 | 90 ± 0 | 99.6 ± 0.0 | 100.2 ± 0.0 | 130.7 ± 0.0 | 98.5 ± 0.0 | 3.88 ± 0.00 | 3.16 ± 0.00 | 0.58 ± 0.00 | 0.35 ± 0.00 | 1.51 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.3 ± 0.0 |
| wave_mu015 | 24 | 1/1 (100%) | 0 | 25.3 ± 0.0 | 103 ± 0 | 99.7 ± 0.0 | 101.5 ± 0.0 | 137.7 ± 0.0 | -2.8 ± 0.0 | 5.21 ± 0.00 | 2.88 ± 0.00 | 0.58 ± 0.00 | 0.95 ± 0.00 | 0.24 ± 0.00 | 0.05 ± 0.00 | 0.06 ± 0.00 | 0.6 ± 0.0 |

Peak joint speed / torque in % of the servo limit (3.65 rad/s, 3.29 N·m at 6.0 V; max over the trials of the condition):

| Condition | ω coxa | ω femur | ω tibia | τ coxa | τ femur | τ tibia |
|---|---:|---:|---:|---:|---:|---:|
| tripod_mu100 | 10% | 18% | 16% | 53% | 86% | 83% |
| wave_mu015 | 38% | 24% | 38% | 21% | 76% | 32% |

Joint torque p95 in % of the limit (mean over trials) and fraction of samples at the limit (>= 98 %) - the peak above is often a single touch-down sample:

| Condition | τ p95 coxa | τ p95 femur | τ p95 tibia | femur at limit | tibia at limit |
|---|---:|---:|---:|---:|---:|
| tripod_mu100 | 53% | 79% | 81% | 0.0% | 0.0% |
| wave_mu015 | 3% | 51% | 19% | 0.0% | 0.0% |

| Condition | Foot sink max (mm) | Foot sink mean (mm) | Slip max (mm) |
|---|---:|---:|---:|
| tripod_mu100 | 0.3 ± 0.0 | -0.0 ± 0.0 | 1.0 ± 0.0 |
| wave_mu015 | 0.1 ± 0.0 | -0.0 ± 0.0 | 2.9 ± 0.0 |
