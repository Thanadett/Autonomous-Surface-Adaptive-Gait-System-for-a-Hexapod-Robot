# E3 calibration - friction and soft-floor checks

| mu | angle (°) | critical atan(mu) (°) | expected | result | moved (mm) |
|---:|---:|---:|---|---|---:|
| 1 | 41.0 | 45.0 | stick | stick | 0 |
| 1 | 49.0 | 45.0 | slide | slide | 376 |
| 0.6 | 27.0 | 31.0 | stick | stick | 0 |
| 0.6 | 35.0 | 31.0 | slide | slide | 555 |
| 0.3 | 12.7 | 16.7 | stick | stick | 0 |
| 0.3 | 20.7 | 16.7 | slide | slide | 709 |
| 0.15 | 4.5 | 8.5 | stick | stick | 0 |
| 0.15 | 12.5 | 8.5 | slide | slide | 793 |

| mat | kp (N/m) | kd (N s/m) | resting penetration (mm) |
|---|---:|---:|---:|
| rigid | - | - | 0.00 |
| soft | 2000.0 | 50.0 | 0.00 |

Friction model as specified: YES.
Soft floor honoured: NO (soft and rigid mats behave alike - report E3 'soft' as rigid-contact limitation)
