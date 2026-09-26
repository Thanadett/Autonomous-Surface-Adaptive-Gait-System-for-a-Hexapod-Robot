"""Recompute E2 metrics from saved samples (no simulation needed).

    ros2 run hexapod_evaluation e2_reevaluate --out results/e2

Reads e2_trials.csv (trial ids, t_command, spec), e2_meta.json (plan, limits),
e2_robot.urdf (the exact model of the run) and samples/<trial>.npz, runs the
current metrics.evaluate() on every trial and rewrites e2_trials.csv (the old
one is kept as e2_trials.prev.csv) and the summary tables. Use it when a metric
definition changes - the trials themselves stay valid.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import sys

from hexapod_kinematics import WholeBodyModel

from .e2_report import write_summary
from .metrics import TrialSamples, TrialSpec, evaluate, foot_radius_from_urdf

KEEP = ("trial", "condition", "group", "speed_label", "rep", "t_command", "warmup_s", "distance_m",
        "stance_m", "step_m", "cadence_scale", "posture_control", "scenario", "surface_mu", "soft",
        "slope_deg", "obstacle_h_m", "ground_x", "ground_y", "ground_z",
        "start_x", "start_y", "start_yaw_deg", "torque_msgs", "contact_msgs", "imu_msgs", "snapshots",
        "start_height_mm", "rtf")


def _num(row: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, "") or default)
    except ValueError:
        return default


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="results/e2")
    args = parser.parse_args(argv)
    with open(os.path.join(args.out, "e2_meta.json"), encoding="utf-8") as handle:
        meta = json.load(handle)
    with open(os.path.join(args.out, "e2_robot.urdf"), encoding="utf-8") as handle:
        xml = handle.read()
    model = WholeBodyModel.from_urdf(xml)
    csv_path = os.path.join(args.out, "e2_trials.csv")
    with open(csv_path, encoding="utf-8") as handle:
        old_rows = list(csv.DictReader(handle))
    plan = meta["plan"]
    rows, missing = [], []
    for old in old_rows:
        path = os.path.join(args.out, "samples", old["trial"] + ".npz")
        if not os.path.exists(path) or "t_command" not in old:
            missing.append(old["trial"])
            continue
        spec = TrialSpec(old["gait"], float(old["speed_cmd_m_s"]), distance_m=float(old["distance_m"]),
                         time_factor=float(plan["time_factor"]), warmup_s=float(old["warmup_s"]),
                         tilt_deg=float(plan["fall"]["tilt_deg"]), min_z_m=float(plan["fall"]["min_z_m"]),
                         slope_deg=_num(old, "slope_deg"),
                         ground_point=(_num(old, "ground_x"), _num(old, "ground_y"), _num(old, "ground_z")),
                         measure_sink='"type": "patch"' in (old.get("scenario") or ""))
        row = {key: old[key] for key in KEEP if key in old}
        row.update(evaluate(TrialSamples.load(path), spec, model, t_command=float(old["t_command"]),
                            foot_radius=foot_radius_from_urdf(xml),
                            effort_limit=float(meta.get("effort_limit_nm", 3.29)),
                            velocity_limit=float(meta.get("velocity_limit_rad_s", 3.65))))
        rows.append(row)
    if missing:
        print(f"no samples / t_command for {len(missing)} trial(s), kept out: {', '.join(missing)}", file=sys.stderr)
    if not rows:
        return 1
    shutil.copyfile(csv_path, os.path.join(args.out, "e2_trials.prev.csv"))
    fields = list(rows[0])
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(write_summary(rows, meta, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
