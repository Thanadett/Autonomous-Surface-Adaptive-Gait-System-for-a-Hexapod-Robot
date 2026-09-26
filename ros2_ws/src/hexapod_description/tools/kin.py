import json,pickle,numpy as np
L=json.load(open('leaves.json')); M=pickle.load(open('meshes.pkl','rb'))
P=np.array([[0,0,1],[1,0,0],[0,1,0]],float)   # SW(x,y,z) -> ROS: x_ros=z_sw, y_ros=x_sw, z_ros=y_sw
base=[l for l in L if l['part']=='base'][0]
bb=np.array(base['bbox']); O=(bb[:3]+bb[3:])/2
def to_ros_pt(p): return P@(np.asarray(p)-O)/1000.0
def to_ros_T(T):
    T=np.array(T); R=P@T[:3,:3]; t=to_ros_pt(T[:3,3]); out=np.eye(4); out[:3,:3]=R; out[:3,3]=t; return out
for l in L:
    l['Tr']=to_ros_T(l['T']); l['comr']=to_ros_pt(l['com'])
    V,F=M[l['i']]; l['V']=(V-O)@P.T/1000.0; l['F']=F
up=np.array([0,0,1.])
servos=[l for l in L if l['part'].startswith('TD')]
for s in servos: s['r']=np.linalg.norm(s['Tr'][:2,3])
servos.sort(key=lambda s:s['r'])
coxa=servos[:6]
def legname(p):
    x,y=p[0],p[1]
    side='L' if y>0 else 'R'
    pos='F' if x>0.03 else ('R' if x<-0.03 else 'M')
    return side+pos
legs={}
for s in coxa: legs[legname(s['Tr'][:3,3])]={'coxa_servo':s}
assert len(legs)==6, legs.keys()
az={k:np.arctan2(*v['coxa_servo']['Tr'][1::-1,3]) for k,v in legs.items()}
def angdiff(a,b): return abs((a-b+np.pi)%(2*np.pi)-np.pi)
def nearest_leg(p):
    a=np.arctan2(p[1],p[0]); return min(az,key=lambda k:angdiff(a,az[k]))
for l in L:
    if l['part'] in('base','top'): l['leg']=None
    else: l['leg']=nearest_leg(l['comr'])
for k,leg in legs.items():
    ss=sorted([s for s in servos if s['leg']==k],key=lambda s:s['r'])
    assert len(ss)==3,(k,len(ss))
    leg['femur_servo'],leg['tibia_servo']=ss[1],ss[2]
def unit(v): v=np.asarray(v,float); return v/np.linalg.norm(v)
def rotm(axis,ang):
    a=unit(axis); K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K
def sang(u,v,a): return np.arctan2(np.dot(np.cross(u,v),a),np.dot(u,v))
for k,leg in legs.items():
    cs,fs,ts=leg['coxa_servo'],leg['femur_servo'],leg['tibia_servo']
    pc=cs['Tr'][:3,3]; xs=cs['Tr'][:3,0]; xm=unit(-(xs-np.dot(xs,up)*up))  # mount outward = -servo long axis
    leg['p_c']=pc; leg['x_mount']=xm
    df=unit(fs['Tr'][:3,2]); pf=fs['Tr'][:3,3]
    xc=unit(np.cross(up,df)); xc=xc-np.dot(xc,up)*up; xc=unit(xc)
    if np.dot(xc,pf-pc)<0: xc=-xc
    A=np.cross(xc,up)   # femur/tibia joint axis (positive = lift)
    n=A
    def proj(p,d): return p+d*(-np.dot(p-pc,n)/np.dot(d,n))
    pf2=proj(pf,df); pt=ts['Tr'][:3,3]; dt=unit(ts['Tr'][:3,2]); pt2=proj(pt,dt)
    ft=[l for l in L if l['leg']==k and l['part']=='foot_tip'][0]
    pfoot=ft['Tr'][:3,:3]@np.array([0,0,0.00075])+ft['Tr'][:3,3]  # fitted sphere centre of foot_tip dome (part-local z=0.75 mm), r = 16.3 mm
    xf=unit(pt2-pf2); xt=unit(pfoot-pt2 - np.dot(pfoot-pt2,A)*A)
    leg.update(x_c=xc,A=A,p_f=pf2,p_t=pt2,p_foot=pfoot,x_f=xf,x_t=xt,
       lat_f=np.dot(pf-pc,A), lat_t=np.dot(pt-pc,A), lat_foot=np.dot(pfoot-pc,A),
       q_coxa=sang(xm,xc,up), q_femur=sang(xc,xf,A), q_tibia=sang(rotm(A,-np.pi/2)@xf,xt,A),
       mount_yaw=np.arctan2(xm[1],xm[0]),
       L_coxa=np.dot(pf2-pc,xc), dz_coxa=np.dot(pf2-pc,up), L_femur=np.linalg.norm(pt2-pf2), L_tibia=np.linalg.norm(pfoot-pt2),
       tib_dot=abs(np.dot(dt,df)))
    # femur link frame at CAD: x=xf, y=-A, z=x×y
def frame(o,x,y):
    x=unit(x); y=unit(y-np.dot(y,x)*x); T=np.eye(4); T[:3,0]=x; T[:3,1]=y; T[:3,2]=np.cross(x,y); T[:3,3]=o; return T
def link_of(l):
    k=l['leg']; p=l['part']
    if k is None: return 'base_link'
    g=legs[k]
    if p.startswith('TD'):
        return {id(g['coxa_servo']):'base_link',id(g['femur_servo']):k+'_coxa',id(g['tibia_servo']):k+'_tibia'}[id(l)]
    if p in('coxa_servo_top','coxa_servo_bottom'): return 'base_link'
    if p in('coxa_top','coxa_bottom','femur_servo_bottom','femur_servo_top'): return k+'_coxa'
    if p in('femur_L','femur_R','connecting_femur'): return k+'_femur'
    if p in('tibia_leg_L','tibia_leg_R'): return k+'_tibia'
    if p in('foot','foot_tip'): return k+'_foot'
    near=min(('coxa','femur','tibia'),key=lambda j:np.linalg.norm(g[j+'_servo']['comr']-l['comr']))
    if p=='femur_servo_mid': return {'femur':k+'_coxa','tibia':k+'_tibia'}[near]
    if p=='attachmentCircular': return {'coxa':k+'_coxa','femur':k+'_femur','tibia':k+'_femur'}[near]
    raise ValueError(p)
for l in L: l['link']=link_of(l)
Tw={'base_link':np.eye(4)}
for k,g in legs.items():
    g['T_mount']=frame(g['p_c'],g['x_mount'],np.cross(up,g['x_mount']))
    Tw[k+'_coxa']=frame(g['p_c'],g['x_c'],-g['A'])
    Tw[k+'_femur']=frame(g['p_f'],g['x_f'],-g['A'])
    Tw[k+'_tibia']=frame(g['p_t'],g['x_t'],-g['A'])
    Tw[k+'_foot']=frame(g['p_foot'],g['x_t'],-g['A'])
