import math, sys, os, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import to_rgb
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lift3d"))
import gen as G, lc as P, render as RD
from meshlib import Part, extrude, rect
plt.rcParams['font.family']='Noto Sans CJK JP'
def xz_frame(theta):
    c,s=math.cos(theta),math.sin(theta)
    return np.stack([np.array([c,0,s]),np.array([s,0,-c]),np.array([0,1.0,0])],1)
def assembly(th):
    xr=P.rod_x(th); zt=P.ZP+P.h_of(th); tip=P.crank_tip(th)
    it=[(P.make_base(),np.eye(3),(0,0,0),"#3a3a3a")]
    it.append((P.make_top_plate(),np.eye(3),(0,0,zt),"#f07c1e"))
    armA=P.make_arm(False); armB=P.make_arm(True)
    for sg in (1,-1):
        R,t=RD.arm_tf((P.XF,P.ZP),(xr,zt),P.A_Y[1],sg); it.append((armA,R,t,"#222222"))
        R,t=RD.arm_tf((xr,P.ZP),(P.XF,zt),P.B_Y[1],sg); it.append((armB,R,t,"#f07c1e"))
    it.append((P.make_bar(*P.BAR_A,"a"),np.eye(3),(xr,0,P.ZP),"#888888"))
    it.append((P.make_bar(-P.BAR_Y,P.BAR_Y,"t"),np.eye(3),(xr,0,zt),"#888888"))
    it.append((P.make_crank(),xz_frame(math.radians(th)),(P.XS,P.CR_PLATE0,P.ZC),"#c8c8c8"))
    phi=math.atan2(P.ZP-tip[1],xr-tip[0])
    it.append((P.make_pushrod(),xz_frame(phi),(tip[0],P.PR_Y0,tip[1]),"#e0e0e0"))
    xc=P.XS-P.SHAFT_OFF
    sv=Part("servo").add(extrude(rect(xc-20.35,P.ZC-9.85,xc+20.35,P.ZC+9.85),[],P.Y_SERVO_BOTTOM,P.Y_SHAFT_END-5.7,"xz"))
    sv.add(extrude(rect(xc-27.25,P.ZC-9.85,xc+27.25,P.ZC+9.85),[],P.Y_EAR,P.Y_EAR+2.5,"xz"))
    it.append((sv,np.eye(3),(0,0,0),"#1f4fa0"))
    return it
def draw(ax,th,title,skip=(),el=22,az=-60,c=(80,0,70),m=105):
    Ts,Cs=[],[]
    Lt=np.array([0.4,-0.6,0.7]);Lt/=np.linalg.norm(Lt)
    for part,R,t,col in assembly(th):
        if part.name in skip: continue
        T=part.transformed(R,t).tris()
        n=np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]); n/=np.linalg.norm(n,axis=1)[:,None]+1e-12
        sh=0.45+0.55*np.abs(n@Lt); Ts.append(T); Cs.append(np.clip(np.array(to_rgb(col))[None]*sh[:,None]*1.1,0,1))
    ax.add_collection3d(Poly3DCollection(np.concatenate(Ts),facecolors=np.concatenate(Cs),linewidths=0))
    ax.set_xlim(c[0]-m,c[0]+m);ax.set_ylim(c[1]-m,c[1]+m);ax.set_zlim(c[2]-m,c[2]+m);ax.set_box_aspect((1,1,1))
    ax.view_init(el,az);ax.set_title(title,fontsize=13);ax.set_axis_off()
fig=plt.figure(figsize=(16,13),dpi=100)
T0,T1=P.TH_DOWN,P.TH_DOWN+P.TH_SPAN
draw(fig.add_subplot(2,2,1,projection='3d'),T0,"내림 — 120×80, 상판 윗면 56.5 mm",c=(60,0,55),m=80)
draw(fig.add_subplot(2,2,2,projection='3d'),T1,"올림 — +62 mm (상판 윗면 118.7 mm)",c=(60,0,55),m=80)
draw(fig.add_subplot(2,2,3,projection='3d'),T0,"구동부 (상판 숨김) — 내림",skip=("top_plate",),el=30,az=-120,c=(50,0,25),m=55)
draw(fig.add_subplot(2,2,4,projection='3d'),78,"구동부 — 중간 (크랭크가 위로 넘어가는 중)",skip=("top_plate",),el=30,az=-120,c=(50,0,35),m=60)
plt.subplots_adjust(left=0,right=1,top=0.96,bottom=0,wspace=0,hspace=0.05)
plt.savefig("preview_c.png")
