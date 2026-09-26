# E4 - ramps, posture control off / on (chapter 4.1.4)

- Generated 2026-09-26 10:24:53; seed 2; course 0.3 m, success = goal within 2.0 x nominal time without a fall (|roll|/|pitch| > 30.0°, z < 60 mm)
- Mass 4.147 kg; cycle times {'tripod': 0.95, 'ripple': 1.3, 'wave': 2.4}; metrics exclude the warm-up (1 gait cycle); Mean ± SD (sample SD) over all trials of a condition
- Ground truth only (OdometryPublisher, joint states, contact and joint F/T sensors); slip = sliding of the foot contact point per stance, centre slip = ball-centre displacement

| Condition | Cmd (mm/s) | Success | Falls | Speed (mm/s) | Speed / cmd (%) | SSM min (mm) | SSM p5 (mm) | SSM mean (mm) | SSM min, contact only (mm) | Support feet | Feet in contact | Slip/stance (mm) | Centre slip (mm) | CoT | Roll RMS (°) | Pitch RMS (°) | Lateral drift (mm) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| tripod_r00_nopc | 25 | 1/1 (100%) | 0 | 22.4 ± 0.0 | 90 ± 0 | 99.6 ± 0.0 | 100.2 ± 0.0 | 130.2 ± 0.0 | 98.5 ± 0.0 | 3.86 ± 0.00 | 3.16 ± 0.00 | 0.58 ± 0.00 | 0.35 ± 0.00 | 1.25 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.3 ± 0.0 |
| wave_r25_pc | 24 | 1/1 (100%) | 0 | 13.1 ± 0.0 | 54 ± 0 | 67.3 ± 0.0 | 69.0 ± 0.0 | 122.6 ± 0.0 | -24.1 ± 0.0 | 5.20 ± 0.00 | 3.14 ± 0.00 | 2.58 ± 0.00 | 2.24 ± 0.00 | 1.73 ± 0.00 | 1.07 ± 0.00 | 16.81 ± 0.00 | 22.6 ± 0.0 |

Peak joint speed / torque in % of the servo limit (3.65 rad/s, 3.29 N·m at 6.0 V; max over the trials of the condition):

| Condition | ω coxa | ω femur | ω tibia | τ coxa | τ femur | τ tibia |
|---|---:|---:|---:|---:|---:|---:|
| tripod_r00_nopc | 10% | 18% | 16% | 100% | 92% | 67% |
| wave_r25_pc | 38% | 32% | 42% | 100% | 100% | 100% |

Joint torque p95 in % of the limit (mean over trials) and fraction of samples at the limit (>= 98 %) - the peak above is often a single touch-down sample:

| Condition | τ p95 coxa | τ p95 femur | τ p95 tibia | femur at limit | tibia at limit |
|---|---:|---:|---:|---:|---:|
| tripod_r00_nopc | 42% | 68% | 65% | 0.0% | 0.0% |
| wave_r25_pc | 26% | 80% | 95% | 0.7% | 5.0% |

| Condition | Body pitch mean (°) | |pitch| max (°) | Roll RMS (°) | Tilt vs slope max (°) | Correction pitch (°) | IMU est. err pitch RMS (°) | IMU est. err roll RMS (°) |
|---|---:|---:|---:|---:|---:|---:|---:|
| tripod_r00_nopc | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.00 ± 0.00 | 0.0 ± 0.0 | 0.0 ± 0.0 | 0.05 ± 0.00 | 0.05 ± 0.00 |
| wave_r25_pc | -16.8 ± 0.0 | 17.5 ± 0.0 | 1.07 ± 0.00 | 9.2 ± 0.0 | -8.2 ± 0.0 | 0.78 ± 0.00 | 0.32 ± 0.00 |
