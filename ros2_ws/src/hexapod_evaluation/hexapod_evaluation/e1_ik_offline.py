"""E1 part A - offline IK accuracy, convergence and timing (no ROS graph needed).

    ros2 run hexapod_evaluation e1_ik_offline --out results/e1
    ros2 run hexapod_evaluation e1_ik_offline --urdf /tmp/hexapod.urdf --samples 2000

For every target both IK solvers of hexapod_kinematics are run - "dls"
(damped least squares, LegKinematics.inverse, what locomotion_node runs
every tick today) and "analytic" (closed form, LegKinematics.inverse_analytic)
- and each solution is put back through FK and compared with the target:

    foot-tip error = |FK(IK(target)) - target|

Target sets (see e1_targets.py): the walking grid (100 per leg) and uniform
joint-space samples over the whole workspace. Two seeds are tested because
they bound what the planner sees: "cold" (all joints 0, worst case) and
"warm" (the nominal-stance solution, which is what the planner seeds with
after its first tick).

Outputs in --out: e1_offline_samples.csv (one row per solve),
e1_offline_summary.md (tables for chapter 4.1.1) and .json.
Timing is wall-clock on the machine that runs it - run it on the RPi 5 as
well to get the number that matters for the real-time budget.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import platform
import subprocess
import sys
import time

import numpy as np

from hexapod_kinematics import LEGS, CadRobotKinematics

from .e1_targets import DEFAULT_STANCE_HEIGHT_M, joint_space_samples, nominal_foot, walking_grid
from .stats import summarize

UPDATE_RATE_HZ = 50.0  # locomotion_node update_rate_hz: one IK per leg per tick


def load_urdf(path: str | None) -> str:
    if path:
        with open(path, encoding="utf-8") as handle:
            return handle.read()
    from ament_index_python.packages import get_package_share_directory

    xacro_file = os.path.join(get_package_share_directory("hexapod_description"), "urdf", "hexapod.urdf.xacro")
    return subprocess.run(
        ["xacro", xacro_file, "use_sim:=false", "use_ros2_control:=false"],
        check=True, capture_output=True, text=True,
    ).stdout


def solve(chain, target, seed, solver: str):
    method = chain.inverse if solver == "dls" else chain.inverse_analytic
    start = time.perf_counter_ns()
    result = method(target, seed)
    elapsed_us = (time.perf_counter_ns() - start) / 1000.0
    error = float(np.linalg.norm(chain.forward(result.angles) - target))
    lower = np.array([joint.lower for joint in chain.joints])
    upper = np.array([joint.upper for joint in chain.joints])
    at_limit = bool(np.any(np.isclose(result.angles, lower, atol=1e-6) | np.isclose(result.angles, upper, atol=1e-6)))
    return result, error, elapsed_us, at_limit


def run(robot: CadRobotKinematics, samples_per_leg: int, seed: int) -> list[dict]:
    rows: list[dict] = []
    rng = np.random.default_rng(seed)
    warm_seeds = {}
    for leg in LEGS:
        warm = robot.legs[leg].inverse(nominal_foot(robot, leg, DEFAULT_STANCE_HEIGHT_M))
        if not warm.converged:
            raise RuntimeError(f"nominal stance unreachable for {leg}")
        warm_seeds[leg] = warm.angles

    sets = {"walking_grid": {leg: (targets, None) for leg, targets in walking_grid(robot).items()}}
    if samples_per_leg > 0:
        sets["joint_space"] = joint_space_samples(robot, samples_per_leg, rng)

    for set_name, per_leg in sets.items():
        for leg, (targets, truths) in per_leg.items():
            chain = robot.legs[leg]
            solvers = ("dls", "analytic") if chain.has_analytic_ik else ("dls",)
            seeds = (("cold", np.zeros(3)), ("warm", warm_seeds[leg]))
            for index, target in enumerate(targets):
                for solver, (seed_name, seed_angles) in itertools.product(solvers, seeds):
                    result, error, elapsed_us, at_limit = solve(chain, target, seed_angles, solver)
                    row = {
                        "set": set_name, "solver": solver, "leg": leg, "index": index, "seed": seed_name,
                        "target_x_m": target[0], "target_y_m": target[1], "target_z_m": target[2],
                        "converged": int(result.converged), "error_m": error,
                        "iterations": result.iterations, "time_us": elapsed_us, "at_limit": int(at_limit),
                        "q_coxa": result.angles[0], "q_femur": result.angles[1], "q_tibia": result.angles[2],
                    }
                    if truths is not None:
                        row["matches_true_q"] = int(np.max(np.abs(result.angles - truths[index])) < 1e-3)
                    rows.append(row)
    return rows


def summarize_rows(rows: list[dict]) -> dict:
    out: dict = {}
    set_names = sorted({r["set"] for r in rows}, reverse=True)
    for set_name, solver, seed_name in itertools.product(set_names, ("dls", "analytic"), ("cold", "warm")):
        subset = [r for r in rows if r["set"] == set_name and r["solver"] == solver and r["seed"] == seed_name]
        if not subset:
            continue
        converged = [r for r in subset if r["converged"]]
        err = summarize(r["error_m"] for r in subset)
        err_conv = summarize(r["error_m"] for r in converged)
        timing = summarize(r["time_us"] for r in subset)
        iters = summarize(r["iterations"] for r in subset)
        entry = {
            "n": len(subset),
            "converged": len(converged),
            "converged_pct": 100.0 * len(converged) / len(subset),
            "error_all_mm": {"mean": err.mean * 1e3, "sd": err.sd * 1e3, "p95": err.p95 * 1e3, "max": err.max * 1e3},
            "error_converged_mm": {"mean": err_conv.mean * 1e3, "sd": err_conv.sd * 1e3, "max": err_conv.max * 1e3},
            "iterations": {"mean": iters.mean, "sd": iters.sd, "max": iters.max},
            "time_us": {"mean": timing.mean, "sd": timing.sd, "p95": timing.p95, "max": timing.max},
            "at_limit": sum(r["at_limit"] for r in subset),
            "per_leg_error_max_mm": {
                leg: max(r["error_m"] for r in subset if r["leg"] == leg) * 1e3 for leg in LEGS
            },
        }
        if "matches_true_q" in subset[0]:
            entry["matches_true_q_pct"] = 100.0 * sum(r["matches_true_q"] for r in subset) / len(subset)
        out[f"{set_name}/{solver}/{seed_name}"] = entry
    return out


def markdown(summary: dict, meta: dict) -> str:
    lines = [
        "# E1 (offline) - IK accuracy, convergence and timing",
        "",
        f"- Generated: {meta['generated']}  |  host: {meta['host']}  |  Python {meta['python']}",
        "- Solvers: dls = LegKinematics.inverse (damped least squares, tol 1e-7 m, max 40 iterations, step <= 0.15 rad, used by locomotion_node); analytic = LegKinematics.inverse_analytic (closed form, 0 iterations)",
        f"- Foot-tip error = |FK(IK(target)) - target|, targets in body_link frame; seeds: cold = 0 rad, warm = nominal stance ({DEFAULT_STANCE_HEIGHT_M * 1000:.0f} mm)",
        "",
        "| Target set / solver / seed | n | Converged | Error mean ± SD (mm) | Error p95 / max (mm) | Iterations mean (max) | Time/solve mean ± SD (µs) | p95 (µs) | At joint limit |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for key, e in summary.items():
        lines.append(
            f"| {key} | {e['n']} | {e['converged']} ({e['converged_pct']:.1f}%) | "
            f"{e['error_all_mm']['mean']:.4f} ± {e['error_all_mm']['sd']:.4f} | "
            f"{e['error_all_mm']['p95']:.4f} / {e['error_all_mm']['max']:.4f} | "
            f"{e['iterations']['mean']:.1f} ({e['iterations']['max']:.0f}) | "
            f"{e['time_us']['mean']:.0f} ± {e['time_us']['sd']:.0f} | {e['time_us']['p95']:.0f} | {e['at_limit']} |"
        )
    budget_ms = 1000.0 / UPDATE_RATE_HZ
    for solver in ("dls", "analytic"):
        grid_warm = summary.get(f"walking_grid/{solver}/warm")
        if grid_warm:
            per_tick_ms = 6 * grid_warm["time_us"]["p95"] / 1000.0
            lines += [
                "",
                f"Real-time budget, {solver} (this host): 6 legs x p95 {grid_warm['time_us']['p95']:.0f} µs = **{per_tick_ms:.2f} ms per tick** "
                f"vs {budget_ms:.0f} ms period at {UPDATE_RATE_HZ:.0f} Hz ({100 * per_tick_ms / budget_ms:.1f}% of the period).",
            ]
    js = [k for k in summary if k.startswith("joint_space")]
    if js:
        lines += ["", "joint_space: same solution as the sampled angles (within 1e-3 rad): " + ", ".join(
            f"{k.split('/', 1)[1]} {summary[k].get('matches_true_q_pct', float('nan')):.1f}%" for k in js)
            + " - a different solution is not an error when the foot-tip error is ~0 (the leg has more than one IK branch)."]
    lines += ["", "Per-leg worst-case error (mm): " + "; ".join(
        f"{k}: " + ", ".join(f"{leg} {v:.4f}" for leg, v in e["per_leg_error_max_mm"].items()) for k, e in summary.items())]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--urdf", help="expanded URDF file (default: xacro hexapod_description)")
    parser.add_argument("--out", default="results/e1", help="output directory")
    parser.add_argument("--samples", type=int, default=2000, help="joint-space samples per leg (0 = skip)")
    parser.add_argument("--seed", type=int, default=1, help="RNG seed for joint-space samples")
    args = parser.parse_args(argv)

    robot = CadRobotKinematics.from_urdf(load_urdf(args.urdf))
    rows = run(robot, args.samples, args.seed)
    summary = summarize_rows(rows)
    meta = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host": f"{platform.node()} ({platform.machine()}, {platform.processor() or platform.system()})",
        "python": platform.python_version(),
        "samples_per_leg": args.samples,
        "rng_seed": args.seed,
    }
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "e1_offline_samples.csv"), "w", newline="", encoding="utf-8") as handle:
        fields = sorted({key for row in rows for key in row}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 99)
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(args.out, "e1_offline_summary.json"), "w", encoding="utf-8") as handle:
        json.dump({"meta": meta, "summary": summary}, handle, indent=2)
    text = markdown(summary, meta)
    with open(os.path.join(args.out, "e1_offline_summary.md"), "w", encoding="utf-8") as handle:
        handle.write(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
