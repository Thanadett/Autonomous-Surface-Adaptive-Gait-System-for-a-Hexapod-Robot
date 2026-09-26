"""Terrain pieces spawned into the running world for E3-E5 (no world file per condition).

One Gazebo session serves a whole experiment: before each trial the robot is parked
away from the test area, the terrain piece of the condition is created with Gazebo's
UserCommands services (`gz service /world/<w>/create`), the robot is teleported onto
it, and after the trial the robot is parked again and the piece removed. Parking first
matters: creating a box under the robot or deleting the ramp it stands on would throw
it around (or drop it 0.3 m).

Scenario dicts (from the plan yaml, see config/e3.yaml ... e5.yaml):
  {"type": "flat"}
  {"type": "patch", "mu": 0.3, "soft": false}          E3: friction / soft-floor mat
  {"type": "ramp", "angle_deg": 15}                     E4: plane rising along +x
  {"type": "obstacle", "height_m": 0.03}                E5: box across the path

Geometry is expressed relative to the nominal start point S = (x0, y0) of the plan
(jitter is applied to the robot only, never to the terrain). Pure functions here
(SDF text, start pose, ground plane) are unit-tested; the gz CLI wrappers are thin.
All CLI calls honour GZ_PARTITION from the environment (scripts/sim_guard.sh).
"""
from __future__ import annotations

import math
import re
import subprocess

# Visual colours: the depth camera cannot see friction, the RGB image can - distinct,
# fixed colours per surface are what a proactive (vision-based) selector learns from.
PATCH_COLOURS = {
    1.0: (0.18, 0.18, 0.20),    # rubber mat, dark grey
    0.6: (0.45, 0.30, 0.15),    # wood / dry soil, brown
    0.3: (0.60, 0.66, 0.72),    # wet tiles, blue-grey
    0.15: (0.86, 0.93, 1.00),   # ice, pale blue-white
}
SOFT_COLOUR = (0.30, 0.50, 0.25)   # foam / grass-like, green
RAMP_COLOUR = (0.55, 0.55, 0.50)
OBSTACLE_COLOUR = (0.90, 0.45, 0.10)

PATCH = {"length": 3.0, "width": 1.2, "thickness": 0.02, "back": 0.4}
RAMP = {"length": 4.0, "width": 1.6, "thickness": 0.20, "back": 0.5, "lift": 0.05, "mu": 0.8}
OBSTACLE = {"depth": 0.20, "width": 1.2, "gap": 0.30, "mu": 0.8}
SOFT = {"mu": 0.8, "kp": 2000.0, "kd": 50.0}   # contact stiffness N/m, damping N s/m (if the engine honours it)


def _colour(mu: float) -> tuple:
    return PATCH_COLOURS.get(round(float(mu), 2), (0.4, 0.4, 0.4))


def box_model_sdf(name: str, size, pose, *, mu: float = 0.8, colour=(0.5, 0.5, 0.5),
                  static: bool = True, mass: float = 1.0, kp: float | None = None,
                  kd: float | None = None) -> str:
    """One-line SDF of a box model (single quotes only, so it fits in a gz text request)."""
    sx, sy, sz = (float(v) for v in size)
    px, py, pz, rr, rp, ry = (float(v) for v in pose)
    contact = ""
    if kp is not None:
        contact = (f"<contact><ode><kp>{kp:g}</kp><kd>{kd if kd is not None else 1.0:g}</kd>"
                   f"<max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact>")
    inertial = ""
    if not static:
        ixx = mass * (sy * sy + sz * sz) / 12.0
        iyy = mass * (sx * sx + sz * sz) / 12.0
        izz = mass * (sx * sx + sy * sy) / 12.0
        inertial = (f"<inertial><mass>{mass:g}</mass><inertia><ixx>{ixx:g}</ixx><iyy>{iyy:g}</iyy>"
                    f"<izz>{izz:g}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>")
    r, g, b = colour
    geometry = f"<geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>"
    return (
        f"<?xml version='1.0'?><sdf version='1.10'><model name='{name}'>"
        f"<static>{'true' if static else 'false'}</static>"
        f"<pose>{px:.6f} {py:.6f} {pz:.6f} {rr:.6f} {rp:.6f} {ry:.6f}</pose>"
        f"<link name='link'>{inertial}"
        f"<collision name='collision'>{geometry}<surface><friction><ode><mu>{mu:g}</mu><mu2>{mu:g}</mu2>"
        f"</ode></friction>{contact}</surface></collision>"
        f"<visual name='visual'>{geometry}<material><ambient>{r} {g} {b} 1</ambient>"
        f"<diffuse>{r} {g} {b} 1</diffuse></material></visual>"
        f"</link></model></sdf>"
    )


def terrain(scenario: dict | None, start_xy, name: str) -> dict:
    """Terrain piece of a scenario: {"sdf", "ground_point", "slope_deg", "start_pitch",
    "describe"}; ground_point is where the robot's feet stand (the start point on top of
    the piece), slope_deg the plane angle along +x."""
    x0, y0 = float(start_xy[0]), float(start_xy[1])
    kind = (scenario or {}).get("type", "flat")
    if kind == "flat":
        return {"sdf": None, "ground_point": (x0, y0, 0.0), "slope_deg": 0.0, "start_pitch": 0.0,
                "describe": "flat"}
    if kind == "patch":
        mu = float(scenario.get("mu", SOFT["mu"]))
        soft = bool(scenario.get("soft", False))
        L, W, T, back = PATCH["length"], PATCH["width"], PATCH["thickness"], PATCH["back"]
        sdf = box_model_sdf(name, (L, W, T), (x0 - back + L / 2, y0, T / 2, 0, 0, 0), mu=mu,
                            colour=SOFT_COLOUR if soft else _colour(mu),
                            kp=float(scenario.get("kp", SOFT["kp"])) if soft else None,
                            kd=float(scenario.get("kd", SOFT["kd"])) if soft else None)
        return {"sdf": sdf, "ground_point": (x0, y0, T), "slope_deg": 0.0, "start_pitch": 0.0,
                "describe": f"patch mu={mu:g}{' soft' if soft else ''}"}
    if kind == "ramp":
        a = math.radians(float(scenario["angle_deg"]))
        L, W, T, back, lift = RAMP["length"], RAMP["width"], RAMP["thickness"], RAMP["back"], RAMP["lift"]
        # start point S on the top surface; the surface reaches down to z = lift 'back' m behind S
        zs = lift + back * math.sin(a)
        t = (math.cos(a), 0.0, math.sin(a))       # up-slope unit vector
        n = (-math.sin(a), 0.0, math.cos(a))      # surface normal
        centre = [s + t[i] * (L / 2 - back) - n[i] * T / 2 for i, s in enumerate((x0, y0, zs))]
        sdf = box_model_sdf(name, (L, W, T), (*centre, 0.0, -a, 0.0), mu=float(scenario.get("mu", RAMP["mu"])),
                            colour=RAMP_COLOUR)
        return {"sdf": sdf, "ground_point": (x0, y0, zs), "slope_deg": math.degrees(a), "start_pitch": -a,
                "describe": f"ramp {math.degrees(a):g} deg"}
    if kind == "obstacle":
        h = float(scenario["height_m"])
        D, W, gap = OBSTACLE["depth"], OBSTACLE["width"], OBSTACLE["gap"]
        sdf = box_model_sdf(name, (D, W, h), (x0 + gap + D / 2, y0, h / 2, 0, 0, 0),
                            mu=OBSTACLE["mu"], colour=OBSTACLE_COLOUR)
        return {"sdf": sdf, "ground_point": (x0, y0, 0.0), "slope_deg": 0.0, "start_pitch": 0.0,
                "describe": f"obstacle {h * 1000:.0f} mm at {gap:g} m",
                "obstacle_x": (x0 + gap, x0 + gap + D)}
    raise ValueError(f"unknown scenario type {kind!r}")


def start_pose(ground_point, slope_deg: float, base_height: float, x: float, y: float, yaw: float):
    """Robot base pose standing on the plane through ground_point (slope along +x):
    position = point of the plane under (x, y) + normal * base_height, orientation
    R = Rz(yaw) Ry(-slope). Returns (x, y, z, qx, qy, qz, qw)."""
    a = math.radians(slope_deg)
    gx, gy, gz = ground_point
    surface_z = gz + (x - gx) * math.tan(a)
    n = (-math.sin(a), 0.0, math.cos(a))
    px, py, pz = x + n[0] * base_height, y + n[1] * base_height, surface_z + n[2] * base_height
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    cy, sy = math.cos(-a / 2), math.sin(-a / 2)
    return px, py, pz, -sz * sy, cz * sy, sz * cy, cz * cy


# -- gz CLI -------------------------------------------------------------------

def _gz_service(args: list[str], timeout_s: float = 15.0) -> tuple[bool, str]:
    try:
        result = subprocess.run(["gz", "service"] + args, capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, str(error)
    output = (result.stdout + result.stderr).strip()
    return result.returncode == 0 and "true" in result.stdout, output


def gz_create(world: str, sdf: str) -> tuple[bool, str]:
    request = 'sdf: "' + sdf.replace('"', "'") + '" allow_renaming: false'
    return _gz_service(["-s", f"/world/{world}/create", "--reqtype", "gz.msgs.EntityFactory",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request])


def gz_remove(world: str, name: str) -> tuple[bool, str]:
    return _gz_service(["-s", f"/world/{world}/remove", "--reqtype", "gz.msgs.Entity",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                        "--req", f'name: "{name}" type: MODEL'])


def gz_set_pose(world: str, model: str, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> tuple[bool, str]:
    request = (f'name: "{model}" position: {{x: {x:.5f} y: {y:.5f} z: {z:.5f}}} '
               f"orientation: {{x: {qx:.8f} y: {qy:.8f} z: {qz:.8f} w: {qw:.8f}}}")
    return _gz_service(["-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request])


def parse_model_position(text: str, name: str):
    """Position (x, y, z) of `name` in a printed gz.msgs.Pose_V (zero fields are omitted
    by the protobuf text format and read as 0). None when the model is not in the text."""
    for block in re.finditer(r"pose\s*\{(.*?)\n\}", text, re.S):
        body = block.group(1)
        if not re.search(r'name:\s*"' + re.escape(name) + r'"', body):
            continue
        match = re.search(r"position\s*\{([^}]*)\}", body)
        values = {"x": 0.0, "y": 0.0, "z": 0.0}
        if match:
            for axis, value in re.findall(r"([xyz]):\s*([-+0-9.eE]+)", match.group(1)):
                values[axis] = float(value)
        return values["x"], values["y"], values["z"]
    return None


def parse_model_names(text: str) -> set[str]:
    """Names of every pose entry in a printed gz.msgs.Pose_V (models and links)."""
    names = set()
    for block in re.finditer(r"pose\s*\{(.*?)\n\}", text, re.S):
        match = re.search(r'name:\s*"([^"]*)"', block.group(1))
        if match:
            names.add(match.group(1))
    return names


def gz_pose_info(world: str, timeout_s: float = 10.0) -> str | None:
    """One message of /world/<w>/pose/info (all entities, static ones included) as text."""
    try:
        result = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"],
                                capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout if "pose" in result.stdout else None


def gz_model_names(world: str, timeout_s: float = 10.0) -> set[str] | None:
    """Entity names currently in the world; None when the pose topic cannot be read."""
    text = gz_pose_info(world, timeout_s)
    return None if text is None else parse_model_names(text)


def gz_model_position(world: str, name: str, timeout_s: float = 10.0):
    try:
        result = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/pose/info"],
                                capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return parse_model_position(result.stdout, name)
