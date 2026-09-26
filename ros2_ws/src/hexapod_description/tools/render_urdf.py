import json, numpy as np, trimesh, yourdfpy, os
from render import render
PKG=os.environ['HEXAPOD_PKG']
def fh(fname): return fname.replace('package://hexapod_description', PKG)
u=yourdfpy.URDF.load('/tmp/claude-0/viz.urdf', filename_handler=fh, build_collision_scene_graph=False)
COL={'petg_grey':(0.78,0.78,0.8),'petg_orange':(0.95,0.45,0.1),'servo_black':(0.15,0.15,0.15),'sensor_blue':(0.2,0.35,0.8)}
cache={}
def items(cfg, collision=False):
    u.update_cfg(cfg); out=[]
    for link in u.link_map.values():
        T=u.get_transform(link.name,'base_link')
        for v in link.visuals:
            g=v.geometry; To=v.origin if v.origin is not None else np.eye(4)
            if g.mesh is not None:
                fn=fh(g.mesh.filename); m=cache.setdefault(fn,trimesh.load(fn))
            elif g.box is not None: m=trimesh.creation.box(g.box.size)
            else: continue
            M=T@To; V=m.vertices@M[:3,:3].T+M[:3,3]
            out.append((V,m.faces,COL.get(v.material.name if v.material else '',(0.7,0.7,0.7))))
    return out
def axes(cfg, names, s=0.03):
    u.update_cfg(cfg); L=[]
    for n in names:
        T=u.get_transform(n,'base_link')
        for i,c in enumerate([(1,0,0),(0,0.7,0),(0,0,1)]): L.append((T[:3,3],T[:3,3]+s*T[:3,i],c))
    return L
if __name__=='__main__':
    q=json.load(open(f'{PKG}/doc/cad_pose_joint_angles.json'))
    N={'LF':'front_left','LM':'middle_left','LR':'rear_left','RF':'front_right','RM':'middle_right','RR':'rear_right'}
    cad={f'{N[k]}_{j}_joint':v[j] for k,v in q.items() for j in v}
    zero={j:0.0 for j in u.actuated_joint_names}
    c=np.array([0,0,-0.05])
    render(items(cad),'urdf_cadpose_iso.png',c+np.array([0.55,0.45,0.4]),c,up=(0,0,1),size=(1200,900))
    render(items(zero),'urdf_zero_iso.png',c+np.array([0.55,0.45,0.35]),c,up=(0,0,1),size=(1200,900))
    fr=['middle_left_coxa_link','middle_left_femur_link','middle_left_tibia_link','middle_left_foot_link','base_link']
    render(items(zero),'urdf_zero_front.png',c+np.array([1.2,0,0]),c,up=(0,0,1),parallel=0.2,size=(1400,800),lines=axes(zero,fr))
