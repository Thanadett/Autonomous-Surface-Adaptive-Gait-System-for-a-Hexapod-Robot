"""Cadence scale for raised steps and non-nominal body heights.

A higher step (step_height) or a different body height (stance_height) makes the
swing legs move faster at the same cycle time - with the velocity-matched swing
and 50 mm steps the femur/tibia would need up to ~2x the servo speed. Instead of
refusing such postures the planner stretches the gait cycle by the factor k below
(cycle_time_used = cycle_time_of(gait) * k). With the stride still capped at
max_stride, the top walking speed drops by 1/k: a high step costs speed, which is
exactly the trade-off the gait selector has to weigh (E2b, E5).

TABLE[gait][i][j] = smallest k that keeps the peak commanded joint speed <= 90 % of
the servo limit (3.65 rad/s) over the whole command envelope (max_linear 0.10 m/s,
max_angular 0.4 rad/s, straight/sideways/diagonal/turning, forward and backward)
at STANCES_M[i] and STEPS_M[j]. None = unreachable (IK fails) - the planner never
asks for those because it clamps the step with max_step_height_for(stance).
Computed offline (25 Sep 2026) on the final1 URDF with gait.yaml's cycle times
(tripod 0.95, ripple 1.30, wave 2.40 s) by tools/cadence_table.py in
hexapod_locomotion; re-run it after changing the geometry, the servo limit, the
swing profile or the cycle times.
"""
from __future__ import annotations

import math

STANCES_M = (0.07, 0.085, 0.1, 0.115, 0.135)
STEPS_M = (0.02, 0.03, 0.04, 0.05)
UNREACHABLE_K = 2.5  # stands in for None cells so interpolation near the limit stays conservative

TABLE = {
    "tripod": (
        (1.028, None, None, None),  # stance 70 mm
        (1.000, 1.253, 1.786, None),  # stance 85 mm
        (1.000, 1.131, 1.582, 2.047),  # stance 100 mm
        (1.000, 1.084, 1.410, 1.823),  # stance 115 mm
        (1.000, 1.122, 1.263, 1.582),  # stance 135 mm
    ),
    "ripple": (
        (1.000, None, None, None),  # stance 70 mm
        (1.000, 1.291, 1.823, None),  # stance 85 mm
        (1.000, 1.188, 1.647, 2.160),  # stance 100 mm
        (1.000, 1.094, 1.496, 1.935),  # stance 115 mm
        (1.000, 1.141, 1.323, 1.690),  # stance 135 mm
    ),
    "wave": (
        (1.019, None, None, None),  # stance 70 mm
        (1.000, 1.345, 1.898, None),  # stance 85 mm
        (1.000, 1.253, 1.748, 2.276),  # stance 100 mm
        (1.000, 1.150, 1.582, 2.047),  # stance 115 mm
        (1.000, 1.188, 1.410, 1.823),  # stance 135 mm
    ),
}


def max_step_height_for(stance_m: float, absolute_max_m: float = 0.05) -> float:
    """Highest step the legs can reach at this body height (IK sweep: 20 mm at 70 mm,
    40 mm at 85 mm, 50 mm from 100 mm up); linear in between, conservative."""
    return float(min(absolute_max_m, max(0.02, 0.02 + (stance_m - 0.07) * (0.02 / 0.015))))


def _index(grid, value):
    value = min(max(value, grid[0]), grid[-1])
    for i in range(len(grid) - 1):
        if value <= grid[i + 1]:
            return i, (value - grid[i]) / (grid[i + 1] - grid[i])
    return len(grid) - 2, 1.0


def cadence_scale(gait: str, stance_m: float, step_m: float) -> float:
    """k >= 1 by bilinear interpolation of TABLE (clamped to the grid)."""
    table = TABLE.get(gait)
    if table is None or not (math.isfinite(stance_m) and math.isfinite(step_m)):
        return 1.0
    i, u = _index(STANCES_M, stance_m)
    j, v = _index(STEPS_M, step_m)

    def cell(a, b):
        value = table[a][b]
        return UNREACHABLE_K if value is None else value

    k = ((1 - u) * (1 - v) * cell(i, j) + u * (1 - v) * cell(i + 1, j)
         + (1 - u) * v * cell(i, j + 1) + u * v * cell(i + 1, j + 1))
    return max(1.0, float(k))
