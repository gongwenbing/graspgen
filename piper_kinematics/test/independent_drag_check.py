#!/usr/bin/env python3
"""Independent SciPy/URDF check of rejected zero-pose drags; no ROS nodes or hardware.

A failed numerical search is not a proof of unreachability.
"""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
root=ET.parse(str(Path(get_package_share_directory('piper_description')) / 'urdf/piper_description.urdf')).getroot()
origins=[];limits=[];axes=[]
for i in range(1,7):
 j=root.find(f"joint[@name='joint{i}']");o=j.find('origin');t=np.eye(4)
 t[:3,:3]=R.from_euler('xyz',list(map(float,o.get('rpy','0 0 0').split()))).as_matrix()
 t[:3,3]=list(map(float,o.get('xyz','0 0 0').split()));origins.append(t)
 axes.append(np.array(list(map(float,j.find('axis').get('xyz').split()))))
 l=j.find('limit');limits.append([float(l.get('lower')),float(l.get('upper'))])
limits=np.array(limits)
def fk(q):
 t=np.eye(4)
 for o,a,v in zip(origins,axes,q):
  motion=np.eye(4);motion[:3,:3]=R.from_rotvec(a*v).as_matrix();t=t@o@motion
 return t
base=fk(np.zeros(6));rng=np.random.default_rng(42);results=[]
for axis,sign in [(0,-1),(0,1),(2,-1),(4,-1)]:
 target=base.copy()
 if axis<3:target[axis,3]+=sign*.005
 else:target[:3,:3]=R.from_rotvec(np.eye(3)[axis-3]*sign*np.deg2rad(2)).as_matrix()@target[:3,:3]
 def error(q):
  actual=fk(q)
  return np.r_[10*(target[:3,3]-actual[:3,3]),R.from_matrix(target[:3,:3]@actual[:3,:3].T).as_rotvec()]
 best=None;feasible=None
 seeds=[np.zeros(6),limits.mean(axis=1)]+[rng.uniform(limits[:,0],limits[:,1]) for _ in range(8)]
 for seed in seeds:
  fit=least_squares(error,seed,bounds=(limits[:,0],limits[:,1]),max_nfev=500,ftol=1e-11,xtol=1e-11,gtol=1e-11)
  e=error(fit.x);pos=np.linalg.norm(e[:3])/10;rot=np.linalg.norm(e[3:]);score=np.dot(e,e)
  item=dict(axis=axis,sign=sign,position_mm=pos*1000,orientation_deg=np.rad2deg(rot),joints=fit.x.tolist(),score=score)
  if best is None or score<best['score']:best=item
  if pos<=.001 and rot<=.01:feasible=item;break
 if feasible is None:
  def excess(q):
   e=error(q)
   p=np.linalg.norm(e[:3])/.01;r=np.linalg.norm(e[3:])/.01
   return np.r_[e[:3]*max(0,1-0.999/max(p,1e-12)),e[3:]*max(0,1-0.999/max(r,1e-12))]
  fit=least_squares(excess,np.array(best['joints']),bounds=(limits[:,0],limits[:,1]),max_nfev=1000,gtol=1e-13,ftol=1e-13,xtol=1e-13)
  e=error(fit.x);pos=np.linalg.norm(e[:3])/10;rot=np.linalg.norm(e[3:])
  if pos<=.001 and rot<=.01:
   feasible=dict(axis=axis,sign=sign,position_mm=pos*1000,orientation_deg=np.rad2deg(rot),joints=fit.x.tolist(),score=float(e@e))
 result=feasible or best;result['accepted']=feasible is not None;results.append(result);print(json.dumps(result),flush=True)
with open('/tmp/graspgen-independent-drag-check.json','w') as f:json.dump(results,f,indent=2)
