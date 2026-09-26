"""Recompute hexapod_locomotion/cadence.py TABLE (see its docstring). Run from ros2_ws:

    python3 src/hexapod_locomotion/tools/cadence_table.py [expanded.urdf]

Prints (gait, stance, step, k) rows and writes cadence_table.json; ~5 min on 2 cores.
"""
import sys, numpy as np, json
from multiprocessing import Pool
sys.path[:0] = ['src/hexapod_kinematics', 'src/hexapod_locomotion']
from hexapod_kinematics import CadRobotKinematics
from hexapod_locomotion import planner as P
r=CadRobotKinematics.from_urdf(open(sys.argv[1] if len(sys.argv) > 1 else 'src/hexapod_description/urdf/hexapod.urdf').read())
CT={'tripod':0.95,'ripple':1.3,'wave':2.4}
BASE=[(0.1,0,0),(0,0.1,0),(0,0,0.4),(0.1,0,0.4),(0.07,0.07,0),(-0.1,0,0),(0.07,0.07,0.4)]
LIM=0.9*3.65
def peak(gait,h,step,k):
    worst=0
    ct={g:v*k for g,v in CT.items()}
    for c in BASE:
        p=P.CadWalkingPlanner(r,P.WalkingSettings(stance_height=h,step_height=step,max_linear_x=0.1,max_linear_y=0.1,cycle_time_by_gait=ct,auto_cadence=False,max_step_height=1.0),gait)
        prev=None
        try:
            for kk in range(int(3*ct[gait]/0.02)+60):
                q=p.update(c,0.02)
                if prev is not None and kk>60: worst=max(worst,np.max(np.abs(q-prev))/0.02)
                prev=q.copy()
        except RuntimeError: return None
    return worst
def cell(args):
    gait,h,step=args
    w=peak(gait,h,step,1.0)
    if w is None:
        w2=peak(gait,h,step,2.0)
        return (gait,h,step,None if w2 is None else 'slowIK')
    if w<=LIM: return (gait,h,step,1.0)
    lo,hi=1.0,1.0
    while True:
        hi*=1.3
        w=peak(gait,h,step,hi)
        if w is None: return (gait,h,step,None)
        if w<=LIM or hi>4: break
    for _ in range(5):
        mid=(lo+hi)/2; w=peak(gait,h,step,mid)
        if w is not None and w<=LIM: hi=mid
        else: lo=mid
    return (gait,h,step,round(hi,3))
jobs=[(g,h,s) for g in CT for h in (0.07,0.085,0.10,0.115,0.135) for s in (0.02,0.03,0.04,0.05)]
with Pool(8) as pool: res=pool.map(cell,jobs)
json.dump(res, open('cadence_table.json', 'w'))
for x in res: print(x)
