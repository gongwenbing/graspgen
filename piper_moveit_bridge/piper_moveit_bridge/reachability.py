"""Necessary geometric bound for the standard PiPER six-joint URDF, not an IK solver."""
import math
import xml.etree.ElementTree as ET


def wrist_reach_bound(pose,urdf_path):
    root=ET.parse(urdf_path).getroot()
    joints=[root.find(f"joint[@name='joint{i}']") for i in range(1,7)]
    if any(j is None for j in joints):return None
    def xyz(j):return [float(v) for v in j.find('origin').get('xyz','0 0 0').split()]
    def norm(v):return math.sqrt(sum(x*x for x in v))
    if norm(xyz(joints[1]))>1e-9 or norm(xyz(joints[4]))>1e-9:return None
    if joints[0].find('parent').get('link')!='base_link':return None
    shoulder=xyz(joints[0]);upper=norm(xyz(joints[2]));forearm=norm(xyz(joints[3]))
    r,p,y=[float(v) for v in joints[5].find('origin').get('rpy','0 0 0').split()]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    rotation=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
              [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    t=xyz(joints[5]);local=[sum(rotation[k][i]*t[k] for k in range(3)) for i in range(3)]
    # Last joint rotates around local Z: the offset's Z component is invariant;
    # its XY part adds a small radius to the possible wrist location.
    if joints[5].find('axis').get('xyz').split()!=['0','0','1']:return None
    q=pose.pose.orientation
    z=[2*(q.x*q.z+q.w*q.y),2*(q.y*q.z-q.w*q.x),1-2*(q.x*q.x+q.y*q.y)]
    position=pose.pose.position
    approximate=[a-b-local[2]*c for a,b,c in zip([position.x,position.y,position.z],shoulder,z)]
    required_min=max(0.,norm(approximate)-math.hypot(local[0],local[1]))
    maximum=upper+forearm
    return {'wrist_distance_min_m':required_min,'main_arm_length_sum_m':maximum,
            'outside_geometric_bound':required_min>maximum+1e-5}
