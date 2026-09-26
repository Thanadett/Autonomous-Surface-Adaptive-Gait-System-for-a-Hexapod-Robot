"""Thesis figures (chapter 3 / 4.1.1-4.1.5) from the simulation results of 25-26 Sep 2026.

    python3 scripts/make_figures.py            # results/ -> results/figures_plots/ (needs matplotlib)

Input: a copy of ros2_ws/results (RES). Output: PNG 300 dpi + PDF (vector) in OUT.
Colours: validated categorical palette (Tripod blue, Ripple orange, Wave aqua) with
markers + line styles as secondary encoding, so the figures survive grey-scale print.
"""
from __future__ import annotations

import csv
import math
import os
import statistics as st
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # ros2_ws (this file: ros2_ws/scripts/)
RES = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WS, "results")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(RES, "figures_plots")
SRC = os.environ.get("HEXAPOD_SRC", os.path.join(WS, "src"))
sys.path[:0] = [f"{SRC}/hexapod_kinematics", f"{SRC}/hexapod_evaluation"]
from hexapod_evaluation.metrics import TrialSamples, body_quantities, quat_to_rpy  # noqa: E402
from hexapod_kinematics import LEGS, WholeBodyModel  # noqa: E402

os.makedirs(OUT, exist_ok=True)
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
GAIT = {"tripod": ("#2a78d6", "o", "Tripod"), "ripple": ("#eb6834", "s", "Ripple"), "wave": ("#1baf7a", "^", "Wave")}
LEG_LABEL = {"front_left": "L1", "middle_left": "L2", "rear_left": "L3",
             "front_right": "R1", "middle_right": "R2", "rear_right": "R3"}
W = 6.3  # figure width in inches (~16 cm, A4 text width)

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 9, "axes.titlesize": 9.5, "axes.labelsize": 9,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "legend.frameon": False, "legend.fontsize": 8.5, "lines.linewidth": 1.8, "lines.markersize": 5.5,
    "savefig.dpi": 300, "savefig.bbox": "tight", "savefig.facecolor": "white",
})


def save(fig, name):
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"))
    plt.close(fig)
    print("wrote", name)


def rows(plan):
    with open(os.path.join(RES, plan, "e2_trials.csv"), encoding="utf-8") as h:
        return list(csv.DictReader(h))


def num(r, k):
    try:
        return float(r[k])
    except (KeyError, TypeError, ValueError):
        return math.nan


def by_cond(rs):
    g = defaultdict(list)
    for r in rs:
        g[r["condition"]].append(r)
    return g


def ms(xs, k, scale=1.0):
    v = [num(x, k) * scale for x in xs if math.isfinite(num(x, k))]
    return (st.mean(v), st.stdev(v) if len(v) > 1 else 0.0) if v else (math.nan, math.nan)


def ok(r):
    return str(r["success"]).lower() in ("true", "1")


# -- E1 -----------------------------------------------------------------------
def fig_e1():
    with open(os.path.join(RES, "e1", "e1_offline_samples.csv"), encoding="utf-8") as h:
        rs = list(csv.DictReader(h))
    fig, (a, b) = plt.subplots(1, 2, figsize=(W, 2.6), gridspec_kw={"width_ratios": [1.6, 1]})
    x = np.arange(len(LEGS))
    for i, (solver, colour, hatch) in enumerate((("dls", "#8a8983", "///"), ("analytic", GAIT["tripod"][0], ""))):
        rate = []
        for leg in LEGS:
            sel = [r for r in rs if r["set"] == "joint_space" and r["solver"] == solver and r["leg"] == leg]
            rate.append(100 * sum(r["converged"] == "1" for r in sel) / len(sel))
        bars = a.bar(x + (i - 0.5) * 0.38, rate, 0.36, color=colour, hatch=hatch, edgecolor="white", linewidth=0.8,
                     label="DLS" if solver == "dls" else "Analytic")
        if solver == "dls":
            for bx, v in zip(bars, rate):
                a.text(bx.get_x() + bx.get_width() / 2, v + 1.5, f"{v:.0f}", ha="center", fontsize=7.5, color=INK2)
    a.set_xticks(x, [LEG_LABEL[l] for l in LEGS])
    a.set_ylim(0, 110)
    a.set_ylabel("Convergence rate (%)")
    a.set_title("(a) Convergence, random joint-space targets")
    a.legend(loc="upper center", ncol=2, bbox_to_anchor=(0.5, 1.0))
    a.set_ylim(0, 120)
    data = []
    for solver in ("dls", "analytic"):
        data.append([float(r["time_us"]) for r in rs if r["set"] == "walking_grid" and r["solver"] == solver
                     and r["seed"] == "warm"])
    bp = b.boxplot(data, tick_labels=["DLS", "Analytic"], widths=0.5, showfliers=False, patch_artist=True,
                   medianprops={"color": INK})
    for patch, c in zip(bp["boxes"], ("#8a8983", GAIT["tripod"][0])):
        patch.set_facecolor(c); patch.set_alpha(0.85); patch.set_edgecolor(INK2)
    for i, d in enumerate(data, start=1):
        b.text(i + 0.3, np.median(d), f"mean {np.mean(d):.0f} µs", ha="left", va="center", fontsize=7.5)
    b.set_xlim(0.5, 2.9)
    b.set_ylim(0, 850)
    b.set_ylabel("Time per IK call (µs)")
    b.set_title("(b) Time per call, walking grid")
    save(fig, "fig_e1_ik_solver")


# -- gait diagram (E2 samples) -------------------------------------------------
def fig_gait_diagram():
    fig, axes = plt.subplots(3, 1, figsize=(W, 3.9), sharex=False)
    for ax, (gait, name) in zip(axes, (("tripod", "tripod_common_01"), ("ripple", "ripple_common_01"),
                                       ("wave", "wave_common_max_01"))):
        s = TrialSamples.load(os.path.join(RES, "e2", "samples", name + ".npz"))
        t = s.t - s.t[0]
        cycle = {"tripod": 0.95, "ripple": 1.30, "wave": 2.40}[gait]
        # two cycles after the warm-up, starting at a swing onset of leg 0
        sw0 = s.swing_plan[:, 0].astype(int)
        on = np.where(np.diff(sw0) == 1)[0]
        on = on[t[on] > 3.0]
        t0 = t[on[0]]
        win = (t >= t0) & (t <= t0 + 2 * cycle)
        order = ["front_left", "middle_left", "rear_left", "front_right", "middle_right", "rear_right"]
        for row, leg in enumerate(order):
            i = LEGS.index(leg)
            swing = s.swing_plan[win, i].astype(bool)
            tt = (t[win] - t0) / cycle
            # stance = filled bar (conventional gait diagram), swing = gap
            k = 0
            while k < len(tt):
                if not swing[k]:
                    j = k
                    while j < len(tt) and not swing[j]:
                        j += 1
                    ax.barh(row, tt[min(j, len(tt) - 1)] - tt[k], left=tt[k], height=0.62, color=GAIT[gait][0])
                    k = j
                else:
                    k += 1
        ax.set_yticks(range(6), [LEG_LABEL[l] for l in order])
        ax.invert_yaxis()
        ax.set_xlim(0, 2)
        ax.grid(axis="y", visible=False)
        duty = {"tripod": "1/2", "ripple": "2/3", "wave": "5/6"}[gait]
        ax.set_title(f"{GAIT[gait][2]} gait  (cycle {cycle:.2f} s, duty factor {duty})", loc="left")
    axes[-1].set_xlabel("Gait cycle  (filled = stance, gap = swing)")
    fig.tight_layout(h_pad=0.6)
    save(fig, "fig_gait_diagram")


# -- E2 -----------------------------------------------------------------------
def fig_e2():
    g = by_cond(rows("e2"))
    conds = [("tripod_common", "25"), ("tripod_max", "100"),
             ("ripple_common", "25"), ("ripple_max", "57"),
             ("wave_common_max", "25")]
    panels = [("speed_m_s", 1000, "Measured speed (mm/s)", "(a) Speed"),
              ("ssm_p5_m", 1000, "SSM, 5th percentile (mm)", "(b) Static stability margin"),
              ("cot", 1, "Cost of transport (-)", "(c) Cost of transport")]
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.7))
    x = np.arange(len(conds))
    for ax, (key, scale, ylabel, title) in zip(axes, panels):
        for i, (c, label) in enumerate(conds):
            gait = c.split("_")[0]
            m, sd = ms(g[c], key, scale)
            hatch = "//" if "max" in c and gait != "wave" else ""
            ax.bar(i, m, 0.7, yerr=sd, color=GAIT[gait][0], hatch=hatch, edgecolor="white", linewidth=0.8,
                   error_kw={"elinewidth": 0.8, "capsize": 2, "ecolor": INK2})
            ax.text(i, m * 1.02 + (0.5 if key != "cot" else 0.01), f"{m:.1f}" if key != "cot" else f"{m:.2f}",
                    ha="center", va="bottom", fontsize=7.5)
        ax.set_xticks(x, [l for _, l in conds], fontsize=8)
        ax.set_xlabel("Commanded speed (mm/s)")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
    axes[0].set_ylim(0, 105)
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=GAIT[k][0], label=GAIT[k][2]) for k in GAIT], ncol=3, loc="upper center",
               bbox_to_anchor=(0.5, 1.06))
    fig.tight_layout()
    save(fig, "fig_e2_gait_performance")


# -- E2b ----------------------------------------------------------------------
def fig_e2b():
    g = by_cond(rows("e2b"))
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.6), sharey=True)
    for ax, body in zip(axes, (100, 135)):
        for gait, (c, mk, name) in GAIT.items():
            xs, ys, es = [], [], []
            for step in (20, 35, 50):
                key = f"{gait}_h{body}_s{step}"
                if key in g:
                    m, sd = ms(g[key], "speed_m_s", 1000)
                    xs.append(step); ys.append(m); es.append(sd)
            ax.errorbar(xs, ys, yerr=es, color=c, marker=mk, label=name, capsize=2)
        ax.axhline(25, color=INK2, lw=0.8, ls=":")
        ax.text(20, 26.2, "command 25 mm/s", fontsize=7.5, color=INK2, ha="left", va="bottom")
        ax.set_xticks((20, 35, 50))
        ax.set_xlabel("Step height (mm)")
        ax.set_title(f"({'a' if body == 100 else 'b'}) Body height {body} mm")
    axes[0].set_ylabel("Measured speed (mm/s)")
    axes[0].set_ylim(0, 30)
    axes[0].legend(loc="lower left")
    fig.tight_layout()
    save(fig, "fig_e2b_step_height")


# -- E3 -----------------------------------------------------------------------
def fig_e3():
    g = by_cond(rows("e3"))
    mus = [(0.15, "mu015"), (0.30, "mu030"), (0.60, "mu060"), (1.00, "mu100")]
    panels = [("speed_ratio", 100, "Speed / command (%)", "(a) Speed"),
              ("slip_mean_m", 1000, "Foot slip per stance (mm)", "(b) Foot slip"),
              ("cot", 1, "Cost of transport (-)", "(c) Cost of transport")]
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.6))
    for ax, (key, scale, ylabel, title) in zip(axes, panels):
        for gait, (c, mk, name) in GAIT.items():
            vals = [ms(g[f"{gait}_{tag}"], key, scale) for _, tag in mus]
            ax.errorbar([m for m, _ in mus], [v[0] for v in vals], yerr=[v[1] for v in vals],
                        color=c, marker=mk, label=name, capsize=2)
        ax.set_xlabel("Friction coefficient μ")
        ax.set_xscale("log")
        ax.set_xticks([0.15, 0.3, 0.6, 1.0], ["0.15", "0.3", "0.6", "1.0"])
        ax.minorticks_off()
        ax.set_ylabel(ylabel)
        ax.set_title(title)
    axes[0].set_ylim(0, 115)
    axes[1].set_ylim(bottom=0)
    axes[2].set_ylim(bottom=0)
    axes[0].legend(loc="lower right")
    fig.tight_layout()
    save(fig, "fig_e3_friction")


# -- E4 -----------------------------------------------------------------------
ANGLES = (0, 5, 10, 15, 20, 25)


def fig_e4_ssm():
    g = by_cond(rows("e4"))
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.6), sharey=True)
    for ax, (gait, (c, mk, name)) in zip(axes, GAIT.items()):
        for pc, ls, fill, label in (("nopc", "--", "white", "Posture control off"), ("pc", "-", c, "Posture control on")):
            vals = [ms(g[f"{gait}_r{a:02d}_{pc}"], "ssm_min_m", 1000) for a in ANGLES]
            ax.errorbar(ANGLES, [v[0] for v in vals], yerr=[v[1] for v in vals], color=c, ls=ls, marker=mk,
                        markerfacecolor=fill, capsize=2, label=label)
        ax.set_title(f"({'abc'[list(GAIT).index(gait)]}) {name}")
        ax.set_xticks(ANGLES)
        ax.set_xlabel("Ramp angle (°)")
        ax.set_ylim(0, 115)
    axes[0].set_ylabel("Minimum SSM (mm)")
    axes[0].legend(loc="lower left", fontsize=7.5)
    fig.tight_layout()
    save(fig, "fig_e4_ssm_vs_slope")


def fig_e4_pitch():
    g = by_cond(rows("e4"))
    fig, ax = plt.subplots(figsize=(W * 0.55, 2.7))
    for pc, ls, label in (("nopc", "--", "Posture control off"), ("pc", "-", "Posture control on")):
        vals = [st.mean(abs(num(r, "pitch_mean_deg")) for gait in GAIT for r in g[f"{gait}_r{a:02d}_{pc}"]) for a in ANGLES]
        ax.plot(ANGLES, vals, ls=ls, marker="o", color=INK if pc == "pc" else INK2,
                markerfacecolor="white" if pc == "nopc" else INK, label=label)
    ax.axhspan(0, 0, color=GRID)
    ax.annotate("gap = maximum correction\n≈ 7.5° (leg reach)", xy=(17.5, 12), xytext=(3.5, 15),
                fontsize=7.5, color=INK2, arrowprops={"arrowstyle": "->", "color": INK2, "lw": 0.7})
    ax.set_xticks(ANGLES)
    ax.set_xlabel("Ramp angle (°)")
    ax.set_ylabel("Body pitch, |mean| (°)")
    ax.set_title("Body pitch on the ramp (mean over the three gaits)")
    ax.legend(loc="upper left")
    save(fig, "fig_e4_body_pitch")


def fig_e4_attitude():
    s = TrialSamples.load(os.path.join(RES, "e4", "samples", "wave_r15_pc_01.npz"))
    t = s.t - s.t[0]
    true_pitch = np.array([math.degrees(quat_to_rpy(q)[1]) for q in s.quat])
    est = np.degrees(s.att_est[:, 1])
    ok_ = np.isfinite(est)
    rms = float(np.sqrt(np.mean((est[ok_] - true_pitch[ok_]) ** 2)))
    fig, ax = plt.subplots(figsize=(W, 2.4))
    ax.plot(t, true_pitch, color=INK, lw=1.6, label="Ground truth (simulator)")
    ax.plot(t, est, color=GAIT["tripod"][0], lw=1.1, ls="--", label="IMU estimate (complementary filter, τ = 0.5 s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Body pitch (°)")
    ax.set_ylim(-9.5, -6)
    ax.set_title(f"Wave gait, 15° ramp, posture control on: body pitch estimate (RMS error {rms:.2f}°)")
    ax.legend(loc="lower left", fontsize=7.5, ncol=2)
    save(fig, "fig_e4_attitude_trace")


# -- E5 -----------------------------------------------------------------------
def fig_e5_ladder():
    rs = rows("e5")
    groups = [("tripod", "nominal"), ("tripod", "high"), ("ripple", "nominal"), ("ripple", "high"),
              ("wave", "nominal"), ("wave", "high")]
    heights = (10, 20, 30)
    fig, ax = plt.subplots(figsize=(W, 2.6))
    width = 0.13
    for j, (gait, post) in enumerate(groups):
        c, mk, name = GAIT[gait]
        for i, h in enumerate(heights):
            sel = [r for r in rs if r["condition"] == f"{gait}_{post}_h{h:03d}"]
            xpos = i + (j - 2.5) * width
            if not sel:
                ax.text(xpos, 3, "–", ha="center", fontsize=8, color=INK2)
                continue
            k = sum(ok(r) for r in sel)
            ax.bar(xpos, 100 * k / len(sel), width * 0.92, color=c if post == "high" else "white",
                   edgecolor=c, linewidth=1.2, hatch="" if post == "high" else "////")
            ax.text(xpos, 100 * k / len(sel) + 2, f"{k}/{len(sel)}", ha="center", fontsize=6.5, rotation=90, va="bottom")
    ax.axhline(80, color=INK2, lw=0.8, ls="--")
    ax.text(2.48, 81, "pass ≥ 4/5", fontsize=7.5, color=INK2, ha="right", va="bottom")
    ax.set_xticks(range(len(heights)), [f"{h} mm" for h in heights])
    ax.set_xlabel("Obstacle height")
    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(0, 112)
    from matplotlib.patches import Patch
    handles = []
    for gait in GAIT:
        c = GAIT[gait][0]
        handles += [Patch(facecolor="white", edgecolor=c, hatch="////", label=f"{GAIT[gait][2]} nominal (100/20 mm)"),
                    Patch(facecolor=c, edgecolor=c, label=f"{GAIT[gait][2]} raised (135/50 mm)")]
    ax.legend(handles=handles, ncol=3, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.2))
    ax.set_title("Obstacle ladder, 0.2 m deep box  (–: not tested, ladder stopped)")
    save(fig, "fig_e5_obstacle_ladder")


def fig_e5_stall():
    model = WholeBodyModel.from_urdf(open(os.path.join(RES, "e4", "e2_robot.urdf"), encoding="utf-8").read())
    fig, (a, b) = plt.subplots(1, 2, figsize=(W, 2.7))
    box = (-5.15, -4.95)
    for name, label, c in (("tripod_nominal_h010_01", "Nominal posture, 10 mm box (fail)", INK2),
                           ("tripod_high_h020_01", "Raised posture, 20 mm box (pass)", GAIT["tripod"][0])):
        s = TrialSamples.load(os.path.join(RES, "e5", "samples", name + ".npz"))
        t = s.t - s.t[0]
        a.plot(t, (s.pos[:, 0] - s.pos[0, 0]) * 1000, color=c, label=label)
        if "nominal" in name:
            centres, _, _ = body_quantities(s, model)
            for leg, ls in (("middle_left", "-"), ("front_left", "--"), ("rear_left", ":")):
                i = LEGS.index(leg)
                b.plot((centres[:, i, 0] + 5.45) * 1000, (centres[:, i, 2] - 0.0163) * 1000, color=INK if leg == "middle_left" else INK2,
                       ls=ls, lw=1.0, label=f"{LEG_LABEL[leg]} foot")
    a.axhline(300, color=INK2, lw=0.7, ls="--")
    a.text(66, 290, "box front edge", fontsize=7.5, color=INK2, ha="right", va="top")
    a.set_xlabel("Time (s)")
    a.set_ylabel("Body progress (mm)")
    a.set_title("(a) Tripod body progress")
    a.axhline(800, color=INK2, lw=0.7, ls=":")
    a.text(1, 810, "goal 0.8 m", fontsize=7.5, color=INK2)
    a.legend(loc="upper left", fontsize=7.5, bbox_to_anchor=(0.0, 0.93))
    b.add_patch(plt.Rectangle((300, 0), 200, 10, color="#d9d8d3", zorder=0))
    b.text(410, 3, "box (10 mm)", ha="center", fontsize=7.5, color=INK2)
    b.set_xlabel("Foot position along the path (mm)")
    b.set_ylabel("Foot sole height (mm)")
    b.set_title("(b) Left feet, nominal posture, 10 mm box")
    b.legend(loc="upper left", fontsize=7.5)
    fig.tight_layout()
    save(fig, "fig_e5_stall_mechanism")


if __name__ == "__main__":
    for f in (fig_e1, fig_gait_diagram, fig_e2, fig_e2b, fig_e3, fig_e4_ssm, fig_e4_pitch, fig_e4_attitude,
              fig_e5_ladder, fig_e5_stall):
        f()
