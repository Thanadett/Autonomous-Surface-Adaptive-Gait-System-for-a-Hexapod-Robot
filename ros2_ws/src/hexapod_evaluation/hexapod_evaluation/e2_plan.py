"""Experiment plans: conditions from config/e*.yaml, gait timing from gait.yaml (no ROS needed).

E2  (e2.yaml):  gait x speed at the nominal posture (body 100 mm, step 20 mm).
E2b (e2b.yaml): gait x posture (body height, step height) at the common speed.
E3  (e3.yaml):  gait x surface patch (friction mu, soft floor) - scenario "patch".
E4  (e4.yaml):  gait x ramp angle x posture control off/on - scenario "ramp".
E5  (e5.yaml):  gait x posture x obstacle height, as a ladder (see ladder_groups).
Speeds are the common speed capped per condition at what the gait reaches in that
posture: a raised step stretches the cycle (hexapod_locomotion/cadence.py), so with
the 50 mm stride cap the top speed falls to max_stride / (duty x cycle x k).
"""
from __future__ import annotations

import os

import yaml


def _share(package: str, *parts: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), *parts)


DUTY = {"tripod": 0.5, "ripple": 2.0 / 3.0, "wave": 5.0 / 6.0}  # hexapod_locomotion/*_gait.py


def load_plan(path: str | None) -> dict:
    """The single top-level entry of the plan file (e2: or e2b: ...), plus its name."""
    with open(path or _share("hexapod_evaluation", "config", "e2.yaml"), encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    name, plan = next(iter(data.items()))
    plan = dict(plan)
    plan.setdefault("name", name)
    return plan


def gait_params(path: str | None = None) -> dict:
    """locomotion_node parameters from hexapod_locomotion/config/gait.yaml (single source of truth)."""
    with open(path or _share("hexapod_locomotion", "config", "gait.yaml"), encoding="utf-8") as handle:
        return yaml.safe_load(handle)["locomotion_node"]["ros__parameters"]


def gait_cycle_times(path: str | None = None) -> dict[str, float]:
    """Per-gait cycle times (nominal posture) from gait.yaml."""
    params = gait_params(path)
    default = float(params.get("cycle_time_s", 0.95))
    return {g: float(params.get(f"cycle_time_{g}_s", 0.0)) or default for g in ("tripod", "ripple", "wave")}


def top_speed(gait: str, cycle_s: float, stance_m: float, step_m: float, max_stride_m: float = 0.05) -> float:
    """Forward speed at which the stride reaches max_stride, with the cadence stretch."""
    from hexapod_locomotion.cadence import cadence_scale

    return max_stride_m / (DUTY[gait] * cycle_s * cadence_scale(gait, stance_m, step_m))


def conditions(plan: dict, gaits: list[str] | None = None, cycles: dict | None = None,
               max_stride_m: float = 0.05) -> list[dict]:
    """E2: (gait, speed) pairs - the common speed for every gait + each gait's top speed.
    E2b (plan has "postures": [[body_m, step_m], ...]): gait x posture at
    min(common speed, 98 % of that posture's top speed)."""
    out = []
    nominal = tuple(plan.get("nominal_posture", (0.10, 0.02)))
    common = float(plan.get("speeds", {}).get("common", 0.025))
    cycles_ = cycles or {"tripod": 0.95, "ripple": 1.30, "wave": 2.40}

    def capped(gait, stance, step):
        top = top_speed(gait, cycles_[gait], stance, step, max_stride_m)
        speed = min(common, 0.98 * top)
        return round(speed, 4), ("common" if speed >= common else "capped")

    if "surfaces" in plan:
        for gait in plan["gaits"]:
            if gaits and gait not in gaits:
                continue
            speed, label = capped(gait, *nominal)
            for surface in plan["surfaces"]:
                scenario = {"type": "patch", "mu": float(surface["mu"]), "soft": bool(surface.get("soft", False))}
                out.append({"gait": gait, "speed": speed, "label": label, "stance": nominal[0], "step": nominal[1],
                            "scenario": scenario, "name": f"{gait}_{surface['name']}"})
        return out
    if "ramps" in plan:
        for gait in plan["gaits"]:
            if gaits and gait not in gaits:
                continue
            speed, label = capped(gait, *nominal)
            for angle in plan["ramps"]["angles_deg"]:
                for posture_control in plan["ramps"].get("posture_control", [False]):
                    out.append({"gait": gait, "speed": speed, "label": label, "stance": nominal[0],
                                "step": nominal[1], "posture_control": bool(posture_control),
                                "scenario": {"type": "ramp", "angle_deg": float(angle)},
                                "name": f"{gait}_r{float(angle):02.0f}_{'pc' if posture_control else 'nopc'}"})
        return out
    if "postures" in plan:
        cycles = cycles or {"tripod": 0.95, "ripple": 1.30, "wave": 2.40}
        common = float(plan["speeds"]["common"])
        for gait in plan["gaits"]:
            if gaits and gait not in gaits:
                continue
            for stance, step in plan["postures"]:
                top = top_speed(gait, cycles[gait], stance, step, max_stride_m)
                speed = min(common, 0.98 * top)
                out.append({"gait": gait, "speed": round(speed, 4), "stance": float(stance), "step": float(step),
                            "label": "common" if speed >= common else "capped",
                            "name": f"{gait}_h{stance * 1000:.0f}_s{step * 1000:.0f}"})
        return out
    for gait in plan["gaits"]:
        if gaits and gait not in gaits:
            continue
        speeds = {"common": float(plan["speeds"]["common"])}
        top = float(plan["speeds"]["max"][gait])
        if abs(top - speeds["common"]) > 1e-9:
            speeds["max"] = top
        else:
            speeds = {"common+max": top}
        for label, speed in speeds.items():
            out.append({"gait": gait, "speed": speed, "label": label, "name": f"{gait}_{label.replace('+', '_')}"})
    return out


def ladder_groups(plan: dict, gaits: list[str] | None = None, cycles: dict | None = None,
                  max_stride_m: float = 0.05) -> list[dict]:
    """E5 obstacle ladders: one group per gait x posture, heights climbed in order.

    A height passes with >= min_success of n trials; the ladder stops at the first
    height that fails. Trials of a failing height stop as soon as its failures make
    min_success unreachable (early_fail = n - min_success + 1), so the last height costs
    ~3 trials instead of 10 - its success rate is then reported as k/m (m < n).
    """
    cycles_ = cycles or {"tripod": 0.95, "ripple": 1.30, "wave": 2.40}
    common = float(plan["speeds"]["common"])
    obstacles = plan["obstacles"]
    out = []
    for gait in plan["gaits"]:
        if gaits and gait not in gaits:
            continue
        for posture_name, (stance, step) in obstacles["postures"].items():
            top = top_speed(gait, cycles_[gait], stance, step, max_stride_m)
            speed = round(min(common, 0.98 * top), 4)
            out.append({"gait": gait, "posture": posture_name, "stance": float(stance), "step": float(step),
                        "speed": speed, "label": "common" if speed >= common else "capped",
                        "heights": [float(h) for h in obstacles["heights_m"]],
                        "min_success": int(obstacles.get("min_success", 8)),
                        "name": f"{gait}_{posture_name}"})
    return out
