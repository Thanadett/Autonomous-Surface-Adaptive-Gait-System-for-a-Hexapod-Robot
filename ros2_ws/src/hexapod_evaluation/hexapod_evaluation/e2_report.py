"""Summary tables of E2/E2b/E3/E4/E5 (chapter 4.1.2-4.1.5) from e2_trials.csv.

    ros2 run hexapod_evaluation e2_report --out results/e2     # re-generate e2_summary.md/.json

Per condition (gait x speed): success k/n, then Mean ± SD over *all* trials of
the condition (sample SD, ddof = 1) - a failed trial still has a speed, SSM,
slip and CoT for the part it walked, and leaving it out would bias the table
towards the good runs. Fall counts are reported separately.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

from .stats import success_rate, summarize

# (column, header, scale, digits)
COLUMNS = [
    ("speed_m_s", "Speed (mm/s)", 1000.0, 1),
    ("speed_ratio", "Speed / cmd (%)", 100.0, 0),
    ("ssm_min_m", "SSM min (mm)", 1000.0, 1),
    ("ssm_p5_m", "SSM p5 (mm)", 1000.0, 1),
    ("ssm_mean_m", "SSM mean (mm)", 1000.0, 1),
    ("ssm_contact_min_m", "SSM min, contact only (mm)", 1000.0, 1),
    ("support_feet_mean", "Support feet", 1.0, 2),
    ("contact_feet_mean", "Feet in contact", 1.0, 2),
    ("slip_mean_m", "Slip/stance (mm)", 1000.0, 2),
    ("centre_slip_mean_m", "Centre slip (mm)", 1000.0, 2),
    ("cot", "CoT", 1.0, 2),
    ("roll_rms_deg", "Roll RMS (°)", 1.0, 2),
    ("pitch_rms_deg", "Pitch RMS (°)", 1.0, 2),
    ("lateral_drift_m", "Lateral drift (mm)", 1000.0, 1),
]
PEAKS = [
    ("qd_peak_coxa_pct", "coxa"), ("qd_peak_femur_pct", "femur"), ("qd_peak_tibia_pct", "tibia"),
]
TORQUES = [
    ("tau_peak_coxa_pct", "coxa"), ("tau_peak_femur_pct", "femur"), ("tau_peak_tibia_pct", "tibia"),
]
# experiment-specific columns (plan key -> columns), shown in an extra table
EXTRA = {
    "surfaces": [("foot_sink_max_m", "Foot sink max (mm)", 1000.0, 1), ("foot_sink_mean_m", "Foot sink mean (mm)", 1000.0, 1),
                 ("slip_max_m", "Slip max (mm)", 1000.0, 1)],
    "ramps": [("pitch_mean_deg", "Body pitch mean (°)", 1.0, 1), ("pitch_abs_max_deg", "|pitch| max (°)", 1.0, 1),
              ("roll_rms_deg", "Roll RMS (°)", 1.0, 2), ("tilt_rel_max_deg", "Tilt vs slope max (°)", 1.0, 1),
              ("posture_pitch_mean_deg", "Correction pitch (°)", 1.0, 1),
              ("att_err_pitch_rms_deg", "IMU est. err pitch RMS (°)", 1.0, 2),
              ("att_err_roll_rms_deg", "IMU est. err roll RMS (°)", 1.0, 2)],
}
TITLES = {
    "e2": "# E2 - basic gaits on flat ground (chapter 4.1.2)",
    "e2b": "# E2b - body height x step height on flat ground (chapter 4.1.2, posture cost)",
    "e3": "# E3 - gait x surface (chapter 4.1.3, Gait Performance Map)",
    "e4": "# E4 - ramps, posture control off / on (chapter 4.1.4)",
    "e5": "# E5 - obstacle height ladder (chapter 4.1.5)",
}
TORQUE_P95 = [
    ("tau_p95_coxa_pct", "coxa"), ("tau_p95_femur_pct", "femur"), ("tau_p95_tibia_pct", "tibia"),
    ("tau_sat_femur_frac", "femur sat"), ("tau_sat_tibia_frac", "tibia sat"),
]


def _float(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _bool(value) -> bool:
    return str(value).strip().lower() in ("true", "1")


def summarize_conditions(rows: list[dict]) -> dict:
    out: dict = {}
    for row in rows:
        out.setdefault(row["condition"], []).append(row)
    summary = {}
    for name, group in out.items():
        entry = {
            "gait": group[0]["gait"], "speed_cmd_m_s": _float(group[0]["speed_cmd_m_s"]),
            "speed_label": group[0].get("speed_label", ""), "n": len(group),
            "success": sum(_bool(r["success"]) for r in group), "falls": sum(_bool(r["fell"]) for r in group),
            "fall_reasons": sorted({r.get("fall_reason", "") for r in group if _bool(r["fell"])}),
        }
        extra_keys = [(k, None) for cols in EXTRA.values() for k, *_ in cols]
        for key, *_ in COLUMNS + [(k, None) for k, _ in PEAKS + TORQUES + TORQUE_P95] + extra_keys + [("time_to_goal_s", None), ("cadence_scale", None)]:
            s = summarize(_float(r.get(key)) for r in group)
            entry[key] = {"n": s.n, "mean": s.mean, "sd": s.sd, "min": s.min, "max": s.max}
        summary[name] = entry
    return summary


def markdown(summary: dict, meta: dict) -> str:
    plan = meta.get("plan", {})
    columns = COLUMNS + ([("cadence_scale", "Cadence k", 1.0, 2)] if "postures" in plan or "obstacles" in plan else [])
    title = TITLES.get(plan.get("name", ""), TITLES["e2b"] if "postures" in plan else TITLES["e2"])
    lines = [
        title,
        "",
        f"- Generated {meta.get('generated', '?')}; seed {meta.get('seed', '?')}; course {meta.get('distance_m', '?')} m, "
        f"success = goal within {meta.get('plan', {}).get('time_factor', 2)} x nominal time without a fall "
        f"(|roll|/|pitch| > {meta.get('plan', {}).get('fall', {}).get('tilt_deg', 30)}°, z < "
        f"{meta.get('plan', {}).get('fall', {}).get('min_z_m', 0.06) * 1000:.0f} mm)",
        f"- Mass {meta.get('mass_kg', math.nan):.3f} kg; cycle times {meta.get('cycle_times_s', {})}; "
        "metrics exclude the warm-up (1 gait cycle); Mean ± SD (sample SD) over all trials of a condition",
        "- Ground truth only (OdometryPublisher, joint states, contact and joint F/T sensors); "
        "slip = sliding of the foot contact point per stance, centre slip = ball-centre displacement",
        "",
        "| Condition | Cmd (mm/s) | Success | Falls | " + " | ".join(h for _, h, _, _ in columns) + " |",
        "|---|---:|---:|---:|" + "---:|" * len(columns),
    ]
    for name, e in summary.items():
        cells = []
        for key, _, scale, digits in columns:
            s = e[key]
            cells.append("n/a" if s["n"] == 0 else f"{s['mean'] * scale:.{digits}f} ± {s['sd'] * scale:.{digits}f}")
        lines.append(f"| {name} | {e['speed_cmd_m_s'] * 1000:.0f} | {success_rate(e['success'], e['n'])} | "
                     f"{e['falls']}{(' (' + ', '.join(e['fall_reasons']) + ')') if e['falls'] else ''} | "
                     + " | ".join(cells) + " |")
    lines += ["", "Peak joint speed / torque in % of the servo limit "
              f"({meta.get('velocity_limit_rad_s', 3.65)} rad/s, {meta.get('effort_limit_nm', 3.29)} N·m at 6.0 V; "
              "max over the trials of the condition):", "",
              "| Condition | ω coxa | ω femur | ω tibia | τ coxa | τ femur | τ tibia |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name, e in summary.items():
        cells = []
        for key, _ in PEAKS + TORQUES:
            s = e[key]
            cells.append("n/a" if s["n"] == 0 else f"{s['max']:.0f}%")
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    lines += ["", "Joint torque p95 in % of the limit (mean over trials) and fraction of samples at the limit "
              "(>= 98 %) - the peak above is often a single touch-down sample:", "",
              "| Condition | τ p95 coxa | τ p95 femur | τ p95 tibia | femur at limit | tibia at limit |",
              "|---|---:|---:|---:|---:|---:|"]
    for name, e in summary.items():
        cells = []
        for key, _ in TORQUE_P95:
            s = e[key]
            if s["n"] == 0:
                cells.append("n/a")
            elif key.startswith("tau_sat"):
                cells.append(f"{100 * s['mean']:.1f}%")
            else:
                cells.append(f"{s['mean']:.0f}%")
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    for key, cols in EXTRA.items():
        if key not in plan:
            continue
        lines += ["", "| Condition | " + " | ".join(h for _, h, _, _ in cols) + " |", "|---|" + "---:|" * len(cols)]
        for name, e in summary.items():
            cells = []
            for col, _, scale, digits in cols:
                st = e.get(col, {"n": 0})
                cells.append("n/a" if st["n"] == 0 else f"{st['mean'] * scale:.{digits}f} ± {st['sd'] * scale:.{digits}f}")
            lines.append(f"| {name} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def ladder_markdown(rows: list[dict], plan: dict) -> str:
    """E5: success k/m per group and height, and the highest height passed (>= min_success)."""
    if "obstacles" not in plan:
        return ""
    need = int(plan["obstacles"].get("min_success", 8))
    table: dict = {}
    for r in rows:
        group = r.get("group") or r["condition"].rsplit("_h", 1)[0]
        height = round(_float(r.get("obstacle_h_m")) * 1000)
        cell = table.setdefault(group, {}).setdefault(height, [0, 0])
        cell[0] += _bool(r["success"])
        cell[1] += 1
    heights = sorted({h for g in table.values() for h in g})
    lines = ["", "## Obstacle ladder: successes / trials per height (pass = at least "
             f"{need}/{plan.get('n', 10)}; a failing height stops early)", "",
             "| Group | " + " | ".join(f"{h} mm" for h in heights) + " | Highest passed |",
             "|---|" + "---:|" * (len(heights) + 1)]
    for group, cells in table.items():
        passed = [h for h, (k, m) in cells.items() if k >= need]
        lines.append(f"| {group} | " + " | ".join(
            f"{cells[h][0]}/{cells[h][1]}" if h in cells else "" for h in heights)
            + f" | {max(passed) if passed else 0} mm |")
    return "\n".join(lines) + "\n"


def write_summary(rows: list[dict], meta: dict, out_dir: str) -> str:
    rows = [{k: (str(v) if isinstance(v, bool) else v) for k, v in r.items()} for r in rows]
    summary = summarize_conditions(rows)
    text = markdown(summary, meta) + ladder_markdown(rows, meta.get("plan", {}))
    with open(os.path.join(out_dir, "e2_summary.md"), "w", encoding="utf-8") as handle:
        handle.write(text)
    with open(os.path.join(out_dir, "e2_summary.json"), "w", encoding="utf-8") as handle:
        json.dump({"meta": meta, "summary": summary}, handle, indent=2, default=str)
    return text


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="results/e2")
    args = parser.parse_args(argv)
    with open(os.path.join(args.out, "e2_trials.csv"), encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    meta = {}
    meta_path = os.path.join(args.out, "e2_meta.json")
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8") as handle:
            meta = json.load(handle)
    print(write_summary(rows, meta, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
