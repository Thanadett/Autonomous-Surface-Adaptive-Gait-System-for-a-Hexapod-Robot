# E2 - basic gaits on flat ground (chapter 4.1.2)

- Generated 2026-09-25 18:00:19; seed 2; course 2.0 m, success = goal within 2.0 x nominal time without a fall (|roll|/|pitch| > 30.0°, z < 60 mm)
- Mass 4.147 kg; cycle times {'tripod': 0.95, 'ripple': 1.3, 'wave': 2.4}; metrics exclude the warm-up (1 gait cycle); Mean ± SD (sample SD) over all trials of a condition
- Ground truth only (OdometryPublisher, joint states, contact and joint F/T sensors); slip = sliding of the foot contact point per stance, centre slip = ball-centre displacement

| Condition | Cmd (mm/s) | Success | Falls | Speed (mm/s) | Speed / cmd (%) | SSM min (mm) | SSM p5 (mm) | SSM mean (mm) | SSM min, contact only (mm) | Support feet | Feet in contact | Slip/stance (mm) | Centre slip (mm) | CoT | Roll RMS (°) | Pitch RMS (°) | Lateral drift (mm) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| tripod_common | 25 | 10/10 (100%) | 0 | 22.4 ± 0.0 | 90 ± 0 | 99.5 ± 0.1 | 100.1 ± 0.0 | 129.0 ± 0.8 | 98.5 ± 0.0 | 3.83 ± 0.02 | 3.16 ± 0.01 | 0.58 ± 0.00 | 0.36 ± 0.01 | 1.25 ± 0.01 | 0.00 ± 0.00 | 0.00 ± 0.00 | 5.1 ± 3.4 |
| tripod_max | 100 | 9/9 (100%) | 0 | 90.0 ± 0.0 | 90 ± 0 | 85.5 ± 0.2 | 87.5 ± 0.1 | 112.1 ± 0.4 | 84.1 ± 0.1 | 3.49 ± 0.01 | 3.16 ± 0.00 | 2.21 ± 0.03 | 1.42 ± 0.03 | 1.02 ± 0.01 | 0.02 ± 0.00 | 0.01 ± 0.00 | 4.9 ± 4.9 |
| ripple_common | 25 | 10/10 (100%) | 0 | 25.4 ± 0.0 | 102 ± 0 | 102.5 ± 0.1 | 103.3 ± 0.0 | 112.5 ± 0.2 | -4.3 ± 0.2 | 4.40 ± 0.02 | 2.74 ± 0.04 | 0.35 ± 0.01 | 0.59 ± 0.01 | 1.02 ± 0.01 | 0.07 ± 0.01 | 0.06 ± 0.00 | 0.8 ± 4.4 |
| ripple_max | 57 | 10/10 (100%) | 0 | 58.2 ± 0.0 | 102 ± 0 | 13.3 ± 29.7 | 99.2 ± 0.2 | 108.3 ± 0.5 | -8.4 ± 0.7 | 4.05 ± 0.02 | 2.72 ± 0.03 | 0.77 ± 0.03 | 1.33 ± 0.02 | 0.98 ± 0.01 | 0.13 ± 0.01 | 0.11 ± 0.01 | 0.1 ± 4.8 |
| wave_common_max | 25 | 10/10 (100%) | 0 | 25.8 ± 0.0 | 103 ± 0 | 99.6 ± 0.0 | 101.5 ± 0.0 | 137.1 ± 0.2 | -6.4 ± 0.2 | 5.20 ± 0.00 | 2.91 ± 0.01 | 0.37 ± 0.02 | 0.69 ± 0.02 | 0.75 ± 0.00 | 0.04 ± 0.00 | 0.05 ± 0.00 | 19.6 ± 5.3 |

Peak joint speed / torque in % of the servo limit (3.65 rad/s, 3.29 N·m at 6.0 V; max over the trials of the condition):

| Condition | ω coxa | ω femur | ω tibia | τ coxa | τ femur | τ tibia |
|---|---:|---:|---:|---:|---:|---:|
| tripod_common | 10% | 18% | 16% | 61% | 100% | 83% |
| tripod_max | 41% | 41% | 47% | 100% | 100% | 100% |
| ripple_common | 17% | 20% | 21% | 100% | 100% | 100% |
| ripple_max | 39% | 43% | 48% | 100% | 100% | 101% |
| wave_common_max | 38% | 24% | 39% | 100% | 100% | 100% |

Joint torque p95 in % of the limit (mean over trials) and fraction of samples at the limit (>= 98 %) - the peak above is often a single touch-down sample:

| Condition | τ p95 coxa | τ p95 femur | τ p95 tibia | femur at limit | tibia at limit |
|---|---:|---:|---:|---:|---:|
| tripod_common | 42% | 68% | 65% | 0.1% | 0.0% |
| tripod_max | 41% | 67% | 64% | 0.2% | 0.1% |
| ripple_common | 40% | 67% | 83% | 1.5% | 0.1% |
| ripple_max | 38% | 66% | 80% | 1.2% | 0.3% |
| wave_common_max | 13% | 100% | 89% | 10.7% | 0.1% |
