# E5 - obstacle height ladder (chapter 4.1.5)

- Generated 2026-09-29 01:37:19; seed 2; course 0.8 m, success = goal within 2.0 x nominal time without a fall (|roll|/|pitch| > 30.0°, z < 60 mm)
- Mass 4.147 kg; cycle times {'tripod': 0.95, 'ripple': 1.3, 'wave': 2.4}; metrics exclude the warm-up (1 gait cycle); Mean ± SD (sample SD) over all trials of a condition
- Ground truth only (OdometryPublisher, joint states, contact and joint F/T sensors); slip = sliding of the foot contact point per stance, centre slip = ball-centre displacement

| Condition | Cmd (mm/s) | Success | Falls | Speed (mm/s) | Speed / cmd (%) | SSM min (mm) | SSM p5 (mm) | SSM mean (mm) | SSM min, contact only (mm) | Support feet | Feet in contact | Slip/stance (mm) | Centre slip (mm) | CoT | Roll RMS (°) | Pitch RMS (°) | Lateral drift (mm) | Cadence k |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| tripod_nominal_h010 | 25 | 0/2 (0%) | 0 | 5.3 ± 0.1 | 21 ± 0 | -6.0 ± 0.1 | -4.0 ± 0.0 | 34.1 ± 0.3 | -53.6 ± 65.7 | 2.96 ± 0.02 | 2.43 ± 0.01 | 1.44 ± 0.00 | 1.13 ± 0.03 | 7.14 ± 0.13 | 0.61 ± 0.00 | 1.21 ± 0.05 | -6.2 ± 1.6 | 1.00 ± 0.00 |
| tripod_high_h010 | 25 | 5/5 (100%) | 0 | 22.8 ± 0.0 | 91 ± 0 | -6.1 ± 0.1 | -1.6 ± 0.1 | 101.7 ± 0.6 | -108.2 ± 0.1 | 3.36 ± 0.01 | 3.04 ± 0.01 | 0.99 ± 0.02 | 0.74 ± 0.03 | 1.95 ± 0.02 | 0.79 ± 0.00 | 1.09 ± 0.00 | 1.0 ± 2.0 | 1.58 ± 0.00 |
| tripod_high_h020 | 25 | 4/5 (80%) | 0 | 12.6 ± 0.2 | 50 ± 1 | -49.1 ± 51.1 | -5.1 ± 0.1 | 65.8 ± 0.6 | -109.1 ± 0.2 | 3.22 ± 0.01 | 2.79 ± 0.01 | 2.06 ± 0.10 | 1.84 ± 0.10 | 5.27 ± 0.06 | 1.42 ± 0.01 | 2.54 ± 0.01 | 116.2 ± 168.1 | 1.58 ± 0.00 |
| tripod_high_h030 | 25 | 0/2 (0%) | 0 | 0.7 ± 0.1 | 3 ± 0 | -114.2 ± 0.2 | -1.8 ± 0.0 | 75.4 ± 1.7 | -262.5 ± 0.1 | 3.30 ± 0.04 | 2.42 ± 0.01 | 6.83 ± 0.17 | 6.45 ± 0.17 | 59.57 ± 6.59 | 0.40 ± 0.01 | 1.34 ± 0.02 | 3.7 ± 14.3 | 1.58 ± 0.00 |
| ripple_nominal_h010 | 25 | 0/2 (0%) | 0 | 5.0 ± 0.2 | 20 ± 1 | -7.5 ± 0.1 | -5.4 ± 0.0 | 11.4 ± 1.0 | -8.6 ± 0.0 | 2.76 ± 0.01 | 2.28 ± 0.00 | 3.64 ± 0.17 | 4.01 ± 0.18 | 5.92 ± 0.24 | 0.53 ± 0.00 | 1.46 ± 0.01 | -2.9 ± 1.6 | 1.00 ± 0.00 |
| ripple_high_h010 | 25 | 5/5 (100%) | 0 | 24.3 ± 0.2 | 97 ± 1 | -9.2 ± 0.3 | -3.7 ± 0.1 | 48.3 ± 1.9 | -105.9 ± 0.3 | 3.28 ± 0.03 | 2.55 ± 0.01 | 1.02 ± 0.02 | 1.67 ± 0.02 | 1.31 ± 0.01 | 0.79 ± 0.01 | 1.33 ± 0.01 | 1.6 ± 5.2 | 1.69 ± 0.00 |
| ripple_high_h020 | 25 | 5/5 (100%) | 0 | 13.9 ± 0.3 | 56 ± 1 | -14.2 ± 0.8 | -6.0 ± 0.6 | 36.2 ± 2.6 | -106.2 ± 0.8 | 3.17 ± 0.02 | 2.59 ± 0.02 | 3.41 ± 0.19 | 3.89 ± 0.16 | 3.36 ± 0.14 | 1.43 ± 0.03 | 3.23 ± 0.04 | 32.2 ± 178.6 | 1.69 ± 0.00 |
| ripple_high_h030 | 25 | 0/2 (0%) | 0 | 5.1 ± 0.1 | 21 ± 0 | -114.4 ± 141.1 | -8.6 ± 0.2 | 18.0 ± 0.5 | -255.1 ± 13.8 | 3.02 ± 0.02 | 2.70 ± 0.01 | 10.29 ± 0.35 | 10.78 ± 0.38 | 10.33 ± 0.25 | 1.51 ± 0.00 | 6.30 ± 0.00 | -3.4 ± 2.2 | 1.69 ± 0.00 |
| wave_nominal_h010 | 24 | 0/2 (0%) | 0 | 5.9 ± 0.0 | 24 ± 0 | -12.8 ± 0.0 | -8.9 ± 0.0 | 19.1 ± 0.6 | -99.2 ± 0.1 | 3.27 ± 0.01 | 2.53 ± 0.00 | 1.75 ± 0.01 | 1.72 ± 0.02 | 4.15 ± 0.00 | 0.38 ± 0.00 | 1.99 ± 0.00 | -9.2 ± 4.3 | 1.00 ± 0.00 |
| wave_high_h010 | 13 | 0/2 (0%) | 0 | 2.9 ± 0.2 | 22 ± 1 | -15.4 ± 0.2 | -7.2 ± 0.1 | 23.4 ± 0.7 | -99.6 ± 0.0 | 3.45 ± 0.02 | 2.67 ± 0.01 | 4.34 ± 0.10 | 4.30 ± 0.06 | 6.20 ± 0.35 | 0.54 ± 0.00 | 2.11 ± 0.00 | 0.6 ± 3.6 | 1.82 ± 0.00 |

Peak joint speed / torque in % of the servo limit (3.65 rad/s, 3.29 N·m at 6.0 V; max over the trials of the condition):

| Condition | ω coxa | ω femur | ω tibia | τ coxa | τ femur | τ tibia |
|---|---:|---:|---:|---:|---:|---:|
| tripod_nominal_h010 | 10% | 23% | 19% | 100% | 101% | 101% |
| tripod_high_h010 | 12% | 38% | 40% | 100% | 102% | 102% |
| tripod_high_h020 | 12% | 69% | 68% | 101% | 103% | 103% |
| tripod_high_h030 | 41% | 171% | 144% | 101% | 195% | 105% |
| ripple_nominal_h010 | 18% | 20% | 24% | 101% | 101% | 100% |
| ripple_high_h010 | 21% | 35% | 36% | 101% | 102% | 102% |
| ripple_high_h020 | 25% | 82% | 85% | 101% | 103% | 103% |
| ripple_high_h030 | 21% | 49% | 50% | 101% | 102% | 101% |
| wave_nominal_h010 | 38% | 47% | 39% | 101% | 102% | 102% |
| wave_high_h010 | 25% | 33% | 36% | 100% | 102% | 102% |

Joint torque p95 in % of the limit (mean over trials) and fraction of samples at the limit (>= 98 %) - the peak above is often a single touch-down sample:

| Condition | τ p95 coxa | τ p95 femur | τ p95 tibia | femur at limit | tibia at limit |
|---|---:|---:|---:|---:|---:|
| tripod_nominal_h010 | 26% | 95% | 80% | 0.8% | 0.5% |
| tripod_high_h010 | 42% | 89% | 63% | 4.1% | 0.2% |
| tripod_high_h020 | 42% | 100% | 70% | 9.1% | 0.4% |
| tripod_high_h030 | 31% | 46% | 66% | 1.2% | 1.8% |
| ripple_nominal_h010 | 10% | 95% | 81% | 2.7% | 0.1% |
| ripple_high_h010 | 26% | 100% | 87% | 8.7% | 0.3% |
| ripple_high_h020 | 29% | 100% | 74% | 13.9% | 0.5% |
| ripple_high_h030 | 24% | 100% | 67% | 13.0% | 0.8% |
| wave_nominal_h010 | 15% | 96% | 81% | 3.3% | 0.9% |
| wave_high_h010 | 9% | 100% | 64% | 25.8% | 0.4% |

## Obstacle ladder: successes / trials per height (pass = at least 4/5; a failing height stops early)

| Group | 10 mm | 20 mm | 30 mm | Highest passed |
|---|---:|---:|---:|---:|
| tripod_nominal | 0/2 |  |  | 0 mm |
| tripod_high | 5/5 | 4/5 | 0/2 | 20 mm |
| ripple_nominal | 0/2 |  |  | 0 mm |
| ripple_high | 5/5 | 5/5 | 0/2 | 20 mm |
| wave_nominal | 0/2 |  |  | 0 mm |
| wave_high | 0/2 |  |  | 0 mm |
