# hexapod_evaluation

Experiment runners and metrics for the simulation test plan (Project doc
"แผนการจำลองและออกแบบการทดสอบ (Simulation)", experiments E1-E7 -> chapter 4.1).
One place for the statistics and target definitions so every result table is
computed the same way.

| Module | Purpose |
|---|---|
| `stats.py` | `summarize()` (n, mean, sample SD, median, p95, max), `success_rate()` -> "9/10 (90%)" |
| `e1_targets.py` | E1 target sets: walking grid (5 x 5 x 4 = 100 per leg around the planner's nominal foot point) and uniform joint-space samples |
| `e1_ik_offline.py` | E1 part A: IK accuracy / convergence / time per solve, DLS vs closed-form, no ROS graph |
| `e1_sim_node.py` + `launch/e1_fixed_base.launch.py` | E1 part B: Gazebo with the body fixed 0.30 m up (`fixed_base:=true`), foot-tip error of IK + controller |
| `metrics.py` | E2-E7 locomotion metrics on logged samples: success/fall, speed, drift, roll/pitch RMS, measured SSM, slip per stance, mechanical CoT, peak joint speed/torque vs the servo limit |
| `e2_plan.py` + `config/e2.yaml` | E2 conditions (gait x speed), cycle times read from hexapod_locomotion's gait.yaml |
| `e2_runner.py` | E2 trials in one running simulation: stop -> MANUAL:gait -> teleport to start +-2 cm/+-3 deg -> walk -> metrics; CSV + summary |
| `e2_report.py` | E2 tables (per condition: k/n, Mean ± SD) from e2_trials.csv |

Run everything for E1 (writes `ros2_ws/results/e1/`):

```bash
bash scripts/run_e1.sh --build     # or --quick for a 5-pose smoke run
```

Definitions used by E1:

- foot-tip error = |FK(q) - target| in body_link, where q is the IK solution
  (part A) or the measured joint positions after settling (part B);
- tracking error = |q_measured - q_commanded| per joint (part B);
- walking grid = nominal foot point (zero-pose x/y, z = -100 mm) + dx, dy in
  {-30, -15, 0, 15, 30} mm, z in {-70, -90, -110, -130} mm - covers
  max_stride 50 mm and stance heights 70-135 mm from gait.yaml.

## E2 - basic gaits on flat ground (Step 2)

```bash
bash scripts/run_e2.sh --build --quick   # 2 trials x 0.5 m per condition, checks the pipeline
bash scripts/run_e2.sh                   # full plan: 5 conditions x 10 trials x 2 m
bash scripts/run_e2.sh -- --gaits wave --n 3 --bag --save-samples
```

Conditions: every gait at the common speed 25 mm/s and at its own top speed
(tripod 100, ripple 62 mm/s; wave's top speed is 25 mm/s) -> 5 conditions.
The simulation runs with `use_camera:=false` (no depth rendering; E2 does not
use the camera) and `flat_world.sdf`.

Definitions (metrics.py docstring has the details):

- success = 2 m along the initial heading, counted after a one-cycle warm-up,
  within 2 x (2 m / commanded speed), without a fall;
- fall = |roll| or |pitch| > 30° or base_link z < 60 mm;
- SSM (measured) = CoM from the measured joint angles and ground-truth
  orientation, support polygon from the feet the contact sensors report;
  `LocomotionState.stability_margin_m` is the *planned* SSM (commanded pose,
  level body) published by locomotion_node;
- slip = sliding of the foot's contact point per stance (ball-foot rotation
  removed, so pure rolling is not slip); centre slip = ball-centre displacement.
  Even ideal kinematics gives ~2 mm (Tripod, 12 mm stride) to ~5-10 mm (50 mm
  stride) of slip, because the planner holds the ball *centre* still while the
  tibia turns - compare Gazebo values with that floor;
- CoT = sum |tau*omega| dt / (m g d), tau = joint F/T sensor torque about the joint axis.
