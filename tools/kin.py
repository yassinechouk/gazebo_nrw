import numpy as np
def R(rpy):
    r,p,y=rpy
    cx,sx=np.cos(r),np.sin(r); cy,sy=np.cos(p),np.sin(p); cz,sz=np.cos(y),np.sin(y)
    return np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
JOINTS = {  # name: (parent, child, xyz, rpy, axis)
 'slider'     : ('base_link','convoyeur', (-1.4318,-6.1731,-1.5024), (1.5708,0,3.1416), (-1,0,0)),
 'pusher'     : ('convoyeur','guidage',   ( 0.93104,0.065,0.08),     (0,1.5708,0),      (0,0,-1)),
 'prismatic x': ('base_link','axe x',     ( 1.1669,-4.9292,2.5233),  (1.5708,0,0),      (0,1,0)),
 'shelf'      : ('axe x','axe y',         ( 0,0,0),                  (0,0,0),           (0,0,-1)),
}
def link_pose(link):
    """return (Rot, trans) of link frame expressed in base_link frame (all joints at 0)"""
    Rt=np.eye(3); t=np.zeros(3)
    chain=[]
    cur=link
    while cur!='base_link':
        j=[k for k,v in JOINTS.items() if v[1]==cur][0]
        chain.append(j); cur=JOINTS[j][0]
    for j in reversed(chain):
        _,_,xyz,rpy,_=JOINTS[j]
        Rj=R(rpy); t = t + Rt@np.array(xyz); Rt = Rt@Rj
    return Rt,t
if __name__=='__main__':
    print(f"{'link':<12} {'origin in base frame':<34} {'joint axis in base frame'}")
    for j,(p,c,xyz,rpy,ax) in JOINTS.items():
        Rp,tp = link_pose(p)
        Rj = Rp@R(rpy); tj = tp + Rp@np.array(xyz)
        aw = Rj@np.array(ax,dtype=float)
        print(f"{c:<12} [{tj[0]:7.3f} {tj[1]:7.3f} {tj[2]:7.3f}]  joint '{j}'  axis_world=[{aw[0]:5.2f} {aw[1]:5.2f} {aw[2]:5.2f}]")
    print()
    for L in ['convoyeur','guidage','axe x','axe y']:
        Rt,t=link_pose(L); print(f"{L:<12} origin=[{t[0]:7.3f} {t[1]:7.3f} {t[2]:7.3f}]  R=\n{np.round(Rt,4)}")
