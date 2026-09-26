"""
Generate meshes + inertial/kinematic properties for hexapod_description
from the analysed STEP assembly (see kin.py / read_step.py).

Output:
  PKG/meshes/visual/<link>_<color>.stl   (link frame, metres)
  PKG/meshes/collision/<link>.stl        (convex hull, link frame)
  PKG/urdf/hexapod_properties.gen.xacro  (all numeric properties; then run tools/mass_budget.py)
  PKG/doc/mass_report.md
"""
import os, json, sys, numpy as np, trimesh, fast_simplification, yaml
exec(open(os.path.join(os.path.dirname(__file__), 'kin.py')).read())

PKG = sys.argv[1] if len(sys.argv) > 1 else 'hexapod_description'
cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), 'mass_config.yaml')))
os.makedirs(f'{PKG}/meshes/visual', exist_ok=True)
os.makedirs(f'{PKG}/meshes/collision', exist_ok=True)
os.makedirs(f'{PKG}/urdf', exist_ok=True)
os.makedirs(f'{PKG}/doc', exist_ok=True)

LEG_ORDER = ['LF', 'LM', 'LR', 'RF', 'RM', 'RR']
KINDS = ['coxa', 'femur', 'tibia', 'foot']

# ---------------------------------------------------------------- mass model
def part_mass(l):
    """Return (mass [kg], density [kg/m^3], source string) for one CAD part."""
    p = l['part']; vol_m3 = l['vol_mm3'] * 1e-9
    if p in cfg['fixed_mass_kg']:
        m = cfg['fixed_mass_kg'][p]; return m, m / vol_m3, 'datasheet/placeholder mass'
    mat = cfg['part_material'].get(p, cfg['default_material'])
    rho = cfg['materials'][mat]['density_kg_m3'] * cfg['materials'][mat].get('fill_factor', 1.0)
    return rho * vol_m3, rho, f'{mat} (rho_eff={rho:.0f} kg/m3)'

for l in L:
    m, rho, src = part_mass(l)
    l['mass'], l['rho'], l['msrc'] = m, rho, src
    I_sw = np.array(l['I']) * 1e-15           # mm^5 -> m^5 (unit density, about part COM, SW axes)
    l['I_ros'] = rho * (P @ I_sw @ P.T)       # kg m^2, ROS world axes

def link_inertial(link):
    parts = [l for l in L if l['link'] == link]
    Ti = np.linalg.inv(Tw[link]); R = Ti[:3, :3]
    m = sum(l['mass'] for l in parts)
    com_w = sum(l['mass'] * l['comr'] for l in parts) / m
    I = np.zeros((3, 3))
    for l in parts:
        d = l['comr'] - com_w
        I += l['I_ros'] + l['mass'] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, R @ com_w + Ti[:3, 3], R @ I @ R.T

inertials = {'base_link': link_inertial('base_link')}
for kind in KINDS:
    vals = [link_inertial(f'{k}_{kind}') for k in LEG_ORDER]
    inertials[kind] = (np.mean([v[0] for v in vals]), np.mean([v[1] for v in vals], 0), np.mean([v[2] for v in vals], 0))
    spread = max(np.linalg.norm(v[1] - inertials[kind][1]) for v in vals)
    print(f'{kind:6s} mass={inertials[kind][0]*1000:7.1f} g  com={np.round(inertials[kind][1]*1000,2)} mm  (leg-to-leg COM spread {spread*1000:.2f} mm)')
print(f"base   mass={inertials['base_link'][0]*1000:7.1f} g  com={np.round(inertials['base_link'][1]*1000,2)} mm")

# ---------------------------------------------------------------- meshes
def color_class(p):
    if p.startswith('TD') or p in ('attachmentCircular', 'femur_servo_mid', 'foot_tip'): return 'dark'
    if p in ('coxa_top', 'coxa_bottom', 'femur_L', 'femur_R', 'foot'): return 'orange'
    return 'grey'

def link_mesh(link, parts_filter=None):
    parts = [l for l in L if l['link'] == link and (parts_filter is None or parts_filter(l))]
    Ti = np.linalg.inv(Tw[link]); ms = []
    for l in parts:
        m = trimesh.Trimesh(l['V'] @ Ti[:3, :3].T + Ti[:3, 3], l['F'], process=True)
        ms.append(m)
    return trimesh.util.concatenate(ms) if ms else None

def decimate(m, ratio, min_faces=1500):
    if len(m.faces) <= min_faces: return m
    red = min(1 - ratio, 1 - min_faces / len(m.faces))
    v, f = fast_simplification.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32), target_reduction=red)
    return trimesh.Trimesh(v, f, process=True)

mesh_link = {'base_link': 'base_link', **{k: f'LF_{k}' for k in KINDS}}   # legs are identical -> one mesh set
visual_files = {}
for name, link in mesh_link.items():
    visual_files[name] = []
    for cc in ('grey', 'orange', 'dark'):
        m = link_mesh(link, lambda l, cc=cc: color_class(l['part']) == cc)
        if m is None: continue
        m = decimate(m, cfg['visual_decimation_ratio'])
        fn = f'{name}_{cc}.stl'; m.export(f'{PKG}/meshes/visual/{fn}')
        visual_files[name].append((fn, cc)); print(f'visual {fn}: {len(m.faces)} faces')
    if name == 'foot': continue   # foot uses a sphere collision
    full = link_mesh(link)
    hull = full.convex_hull
    hull.export(f'{PKG}/meshes/collision/{name}.stl'); print(f'collision {name}.stl: {len(hull.faces)} faces')

# ---------------------------------------------------------------- kinematics (identical for all legs)
def rpy(R):
    sy = -R[2, 0]
    pitch = np.arcsin(np.clip(sy, -1, 1))
    if abs(abs(sy) - 1) < 1e-9: return 0.0, pitch, np.arctan2(-R[0, 1], R[1, 1])
    return np.arctan2(R[2, 1], R[2, 2]), pitch, np.arctan2(R[1, 0], R[0, 0])

def origin(Tp, Tc, axis_c, q):
    Rq = np.eye(4); Rq[:3, :3] = rotm(axis_c, -q)
    return np.linalg.inv(Tp) @ Tc @ Rq

kin = {}
for k in LEG_ORDER:
    g = legs[k]
    kin[k] = dict(
        mount=g['T_mount'],
        femur=origin(Tw[k + '_coxa'], Tw[k + '_femur'], [0, -1, 0], g['q_femur']),
        tibia=origin(Tw[k + '_femur'], Tw[k + '_tibia'], [0, -1, 0], g['q_tibia']),
        foot=np.linalg.inv(Tw[k + '_tibia']) @ Tw[k + '_foot'],
        coxa_chk=np.linalg.inv(g['T_mount']) @ Tw[k + '_coxa'])
    # sanity: coxa link at CAD = mount * Rz(q_coxa)
    assert np.allclose(kin[k]['coxa_chk'][:3, :3], rotm([0, 0, 1], g['q_coxa']), atol=1e-6)
avg = {j: np.mean([kin[k][j] for k in LEG_ORDER], 0) for j in ('femur', 'tibia', 'foot')}
for j in avg:
    dev = max(np.abs(kin[k][j][:3, 3] - avg[j][:3, 3]).max() for k in LEG_ORDER)
    print(f'{j} joint origin xyz(mm)={np.round(avg[j][:3,3]*1000,3)} rpy(deg)={np.round(np.degrees(rpy(avg[j][:3,:3])),3)} max leg dev={dev*1000:.3f} mm')

foot_tip = [l for l in L if l['link'] == 'LF_foot' and l['part'] == 'foot_tip'][0]
ftV = foot_tip['V']; center = Tw['LF_foot'][:3, 3]
# least-squares sphere fit to the dome of foot_tip, in the part's own CAD frame (mm)
Tft = np.array(foot_tip['T']); Vl = (M[foot_tip['i']][0] - Tft[:3, 3]) @ Tft[:3, :3]
dome = Vl[Vl[:, 2] < -2.0]
sol = np.linalg.lstsq(np.c_[2 * dome, np.ones(len(dome))], (dome ** 2).sum(1), rcond=None)[0]
foot_radius = float(np.sqrt(sol[3] + sol[:3] @ sol[:3])) / 1000.0
print('foot_tip sphere centre (part-local, mm)', np.round(sol[:3], 3), '(kin.py uses z=0.75)')
print('foot sphere radius (m)', foot_radius)

# body extents for sensor placeholders
bV = np.vstack([l['V'] for l in L if l['part'] in ('base', 'top')])
body_min, body_max = bV.min(0), bV.max(0)
print('body bbox (mm)', np.round(body_min * 1000, 1), np.round(body_max * 1000, 1))

# ---------------------------------------------------------------- write properties xacro
def f(x):
    x = float(x)
    if abs(x) < 1e-11: x = 0.0
    return f'{x + 0.0:.6g}'
def v3(a): return ' '.join(f(x) for x in a)
def inert_xml(name, m, c, I):
    return (f'  <xacro:property name="{name}_mass" value="{f(m)}"/>\n'
            f'  <xacro:macro name="{name}_inertial">\n'
            f'    <inertial>\n'
            f'      <origin xyz="{v3(c)}" rpy="0 0 0"/>\n'
            f'      <mass value="{f(m)}"/>\n'
            f'      <inertia ixx="{f(I[0,0])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyy="{f(I[1,1])}" iyz="{f(I[1,2])}" izz="{f(I[2,2])}"/>\n'
            f'    </inertial>\n'
            f'  </xacro:macro>\n')

x = ['<?xml version="1.0"?>',
     '<!-- AUTO-GENERATED by tools/gen.py from hexapod_final1_export.STEP (final1.SLDASM). Do not edit by hand;',
     '     edit tools/mass_config.yaml and re-run the generator instead. Units: m, kg, rad. -->',
     '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">', '',
     '  <!-- ===== Leg mounting (base_link -> <leg>_coxa joint origin) ===== -->']
for k in LEG_ORDER:
    T = kin[k]['mount']; r = rpy(T[:3, :3])
    x.append(f'  <xacro:property name="{k}_mount_xyz" value="{v3(T[:3,3])}"/> <xacro:property name="{k}_mount_rpy" value="{v3(r)}"/>')
x.append('\n  <!-- ===== Leg kinematics (identical for all six legs; max deviation between legs < 0.1 mm) ===== -->')
for j in ('femur', 'tibia', 'foot'):
    T = avg[j]; r = rpy(T[:3, :3])
    x.append(f'  <xacro:property name="{j}_joint_xyz" value="{v3(np.round(T[:3,3],6))}"/> <xacro:property name="{j}_joint_rpy" value="{v3(np.round(r,6))}"/>')
x.append(f'  <xacro:property name="foot_radius" value="{f(foot_radius)}"/>')
x.append('\n  <!-- ===== Inertial properties (link frame). See doc/mass_report.md for assumptions ===== -->')
x.append(inert_xml('base', *inertials['base_link']))
for kind in KINDS: x.append(inert_xml(kind, *inertials[kind]))
x.append('  <!-- ===== Body extents (for sensor placeholder placement) ===== -->')
x.append(f'  <xacro:property name="body_min" value="{v3(body_min)}"/>')
x.append(f'  <xacro:property name="body_max" value="{v3(body_max)}"/>')
x.append(f'  <xacro:property name="body_front_x" value="{f(body_max[0])}"/> <xacro:property name="body_top_z" value="{f(body_max[2])}"/>')
x.append('\n  <!-- ===== Visual mesh groups ===== -->')
for name, files in visual_files.items():
    x.append(f'  <!-- {name}: ' + ', '.join(fn for fn, _ in files) + ' -->')
x.append('</robot>\n')
open(f'{PKG}/urdf/hexapod_properties.gen.xacro', 'w').write('\n'.join(x))
json.dump({k: [fn for fn, _ in v] for k, v in visual_files.items()}, open(f'{PKG}/doc/visual_files.json', 'w'))

# ---------------------------------------------------------------- mass report
rows = {}
for l in L:
    key = (l['part'], l['msrc']); r = rows.setdefault(key, [0, 0, 0]); r[0] += 1; r[1] = l['vol_mm3'] / 1000; r[2] = l['mass']
tot = sum(l['mass'] for l in L)
rep = ['# Mass report (auto-generated)', '',
       f'Total estimated mass (CAD only, no fasteners/electronics): **{tot*1000:.0f} g**', '',
       '| Part | Qty | Volume (cm³) | Mass/pc (g) | Source |', '|---|---:|---:|---:|---|']
for (p, s), (n, v, m) in sorted(rows.items(), key=lambda kv: -kv[1][0] * kv[1][2]):
    rep.append(f'| {p} | {n} | {v:.2f} | {m*1000:.1f} | {s} |')
rep += ['', '| Link | Mass (g) | COM in link frame (mm) |', '|---|---:|---|',
        f"| base_link | {inertials['base_link'][0]*1000:.1f} | {np.round(inertials['base_link'][1]*1000,1).tolist()} |"]
for kind in KINDS:
    rep.append(f'| <leg>_{kind} | {inertials[kind][0]*1000:.1f} | {np.round(inertials[kind][1]*1000,1).tolist()} |')
open(f'{PKG}/doc/mass_report.md', 'w').write('\n'.join(rep) + '\n')
print('TOTAL mass %.1f g' % (tot * 1000))

# CAD pose joint angles (for reference / regression test)
json.dump({k: dict(coxa=legs[k]['q_coxa'], femur=legs[k]['q_femur'], tibia=legs[k]['q_tibia']) for k in LEG_ORDER},
          open(f'{PKG}/doc/cad_pose_joint_angles.json', 'w'), indent=1)
