"""Verify the generated URDF against the CAD geometry.
Usage (after xacro -> /tmp/.../viz.urdf):  HEXAPOD_PKG=<pkg dir> python3 verify.py
1) Forward kinematics at the CAD joint angles must reproduce every CAD link frame.
2) URDF visual meshes posed at the CAD angles must lie on the original CAD surfaces.
"""
import trimesh
import json, numpy as np, yourdfpy, os, pickle
exec(open('kin.py').read())
PKG=os.environ['HEXAPOD_PKG']
# CAD/generator short leg names -> URDF (ros2_ws) names
N={'LF':'front_left','LM':'middle_left','LR':'rear_left','RF':'front_right','RM':'middle_right','RR':'rear_right'}
def ul(k,lk): return f'{N[k]}_{lk}_link'
def fh(fname): return fname.replace('package://hexapod_description', PKG)
u=yourdfpy.URDF.load('/tmp/claude-0/viz.urdf', filename_handler=fh, load_meshes=True, build_collision_scene_graph=False)
print('actuated joints:', len(u.actuated_joint_names))
q=json.load(open(f'{PKG}/doc/cad_pose_joint_angles.json'))
cfg={}
for k,v in q.items():
    for j in ('coxa','femur','tibia'): cfg[f'{N[k]}_{j}_joint']=v[j]
u.update_cfg(cfg)
errs=[]
for k in legs:
    for lk in ('coxa','femur','tibia','foot'):
        T=u.get_transform(ul(k,lk),'base_link')
        errs.append((f'{k}_{lk}', np.linalg.norm(T[:3,3]-Tw[f'{k}_{lk}'][:3,3])*1000, np.degrees(np.arccos(np.clip((np.trace(T[:3,:3].T@Tw[f'{k}_{lk}'][:3,:3])-1)/2,-1,1)))))
print('max frame position error at CAD pose: %.4f mm, max rotation error %.4f deg'%(max(e[1] for e in errs),max(e[2] for e in errs)))
# mesh-level check (surface distance, legs LF and RR)
worst={}
for k in ('LF','RR'):
    for lk in ('coxa','femur','tibia','foot'):
        link=f'{k}_{lk}'
        parts=[l for l in L if l['link']==link]
        cad=trimesh.util.concatenate([trimesh.Trimesh(l['V'],l['F']) for l in parts])
        T=u.get_transform(ul(k,lk),'base_link')
        vis=np.vstack([trimesh.load(fh(v.geometry.mesh.filename)).vertices for v in u.link_map[ul(k,lk)].visuals])
        vis=(vis@T[:3,:3].T+T[:3,3])
        idx=np.random.default_rng(0).choice(len(vis),min(3000,len(vis)),replace=False)
        _,d,_=trimesh.proximity.closest_point(cad,vis[idx])
        print(link,'max %.3f mm  p99 %.3f mm'%(d.max()*1000,np.percentile(d,99)*1000))
# zero pose facts
u.update_cfg({j:0.0 for j in u.actuated_joint_names})
f=u.get_transform('middle_left_foot_link','base_link')[:3,3]; print('zero pose LM foot centre (mm):',np.round(f*1000,1),' ground clearance below base_link = %.1f mm'%(-(f[2]-0.0163182)*1000))
m=sum(l.inertial.mass for l in u.robot.links if l.inertial); print('total URDF mass %.3f kg'%m)
