import math, sys, os, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lift3d"))
import lc as C
from check import Box, SlabCapsule, cyl_y
def cy(x,z,r,y0,y1,n): return SlabCapsule((x,z),(x,z),r,y0,y1,"xz",n,False)
W=C.BW/2; YW=C.Y_WALL
def bodies(th):
    xr=C.rod_x(th); h=C.h_of(th); zt=C.ZP+h; ZP=C.ZP; tip=C.crank_tip(th)
    mid=(C.XF+(xr-C.XF)/2, ZP+h/2); B={}
    B["crank_hub"]=cy(C.XS,C.ZC,9.5,C.CR_PLATE0,C.CR_PLATE1,"crank hub")
    B["crank_arm"]=SlabCapsule((C.XS,C.ZC),tip,5.5,C.CR_PLATE0,C.CR_PLATE1,"xz","crank arm",False)
    B["crank_boss"]=cy(C.XS,C.ZC,7.0,C.CR_BOSS0,C.CR_PLATE0,"crank boss")
    B["tip_head"]=cy(*tip,4.6,C.CR_PLATE0-2.5,C.CR_PLATE0,"tip pin head")
    B["pushrod"]=SlabCapsule(tip,(xr,ZP),5.8,C.PR_Y0,C.PR_Y1,"xz","pushrod",False)
    B["barA"]=Box(xr-6,xr+6,C.BAR_A[0],C.BAR_A[1],ZP-6,ZP+6,"bottom bar")
    B["b_spacer"]=cyl_y(xr,ZP,5.5,C.B_Y[1]+0.25,YW-0.25,"bottom spacer")
    B["b_rod_end"]=cyl_y(xr,ZP,5.7,W,W+5.7,"bottom rod head/clip")
    B["A"]=SlabCapsule((C.XF,ZP),(xr,zt),7,*C.A_Y,"xz","arm A",True)
    B["B"]=SlabCapsule((xr,ZP),(C.XF,zt),7,*C.B_Y,"xz","arm B",True)
    B["mid_head"]=cyl_y(*mid,5.1,C.A_Y[1],C.A_Y[1]+2.6,"middle head")
    B["fix_clip_b"]=cyl_y(C.XF,ZP,5.7,C.A_Y[0]-3.5,C.A_Y[0],"bottom fixed clip")
    B["fix_head_b"]=cyl_y(C.XF,ZP,5.1,W,W+2.6,"bottom fixed head")
    B["fix_clip_t"]=cyl_y(C.XF,zt,5.7,C.B_Y[0]-3.5,C.B_Y[0],"top fixed clip")
    B["fix_head_t"]=cyl_y(C.XF,zt,5.1,W,W+2.6,"top fixed head")
    B["t_spacer"]=cyl_y(xr,zt,5.5,C.BAR_Y+0.25,C.A_Y[0]-0.25,"top spacer")
    B["t_rod_end"]=cyl_y(xr,zt,5.7,W,W+5.7,"top rod end")
    B["tbar"]=Box(xr-6,xr+6,-C.BAR_Y,C.BAR_Y,zt-6,zt+6,"top bar")
    zu=zt+C.TOP_ROD_BELOW
    B["plate"]=Box(0,C.BL,-W,W,zu,zu+C.TOP_T,"top plate")
    B["flangeL"]=Box(0,C.BL,YW,W,zu-C.FLANGE_D,zu,"flange")
    B["flangeR"]=Box(0,C.BL,-W,-YW,zu-C.FLANGE_D,zu,"flange")
    xc=C.XS-C.SHAFT_OFF
    B["servo"]=Box(xc-20.35,xc+20.35,C.Y_SERVO_BOTTOM,C.Y_SHAFT_END-5.7,C.ZC-9.85,C.ZC+9.85,"servo body")
    B["ears"]=Box(xc-27.25,xc+27.25,C.Y_EAR,C.Y_EAR+2.5,C.ZC-9.85,C.ZC+9.85,"servo ears")
    B["spline"]=cy(C.XS,C.ZC,3.0,C.Y_SHAFT_END-5.7,C.CR_BOSS0+0.5,"spline")
    x0,x1=xc-C.SERVO_L/2-0.3,xc+C.SERVO_L/2+0.3; z1=C.ZC+C.SERVO_W/2+0.3; yw0,yw1=C.Y_EAR-4,C.Y_EAR
    B["swall_L"]=Box(3.5,x0,yw0,yw1,0,z1+3.5,"servo wall")
    B["swall_R"]=Box(x1,xc+28.75,yw0,yw1,0,z1+3.5,"servo wall")
    B["swall_T"]=Box(x0,x1,yw0,yw1,z1,z1+3.5,"servo wall")
    B["sstop"]=Box(x0+3,x1-3,C.Y_SERVO_BOTTOM-2.5,C.Y_SERVO_BOTTOM-0.5,2.7,C.ZC+4,"servo stop")
    B["floor"]=Box(0,C.BL,-W,W,0,3,"floor")
    B["wallL"]=Box(0,C.BL,YW,W,0,C.WALL_H,"side wall")
    B["wallR"]=Box(0,C.BL,-W,-YW,0,C.WALL_H,"side wall")
    B["front_low"]=Box(0,4,-YW,YW,0,6,"front wall")
    B["front_mid"]=Box(0,4,-C.B_Y[0],C.B_Y[0],0,C.WALL_H,"front wall")
    B["rear"]=Box(C.BL-4,C.BL,-YW,YW,0,6,"rear wall")
    return B
JOINED={frozenset(p) for p in [
 ("crank_hub","crank_arm"),("crank_hub","crank_boss"),("crank_arm","crank_boss"),("crank_boss","spline"),("crank_hub","spline"),
 ("crank_arm","tip_head"),("crank_hub","tip_head"),("pushrod","tip_head"),
 ("pushrod","barA"),("barA","B"),("b_spacer","B"),("b_spacer","wallL"),("b_spacer","wallR"),("b_spacer","b_rod_end"),("b_rod_end","wallL"),("b_rod_end","wallR"),
 ("A","B"),("A","mid_head"),("B","mid_head"),("mid_clip","B"),("mid_clip","mid_head"),("A","fix_clip_b"),("A","fix_head_b"),
 ("fix_head_b","wallL"),("fix_head_b","wallR"),("B","fix_clip_t"),("B","fix_head_t"),("fix_head_t","flangeL"),("fix_head_t","flangeR"),
 ("t_spacer","A"),("t_spacer","B"),("t_spacer","tbar"),("t_rod_end","flangeL"),("t_rod_end","flangeR"),("tbar","A"),("plate","flangeL"),("plate","flangeR"),
 ("A","wallL"),("A","wallR"),("B","flangeL"),("B","flangeR"),("A","flangeL"),("A","flangeR"),("pushrod","B"),
 ("servo","ears"),("servo","spline"),("ears","swall_L"),("ears","swall_R"),("ears","swall_T"),("servo","swall_L"),("servo","swall_R"),
 ("servo","sstop"),("sstop","floor"),("swall_L","floor"),("swall_R","floor"),("swall_L","swall_T"),("swall_R","swall_T"),("swall_L","front_low"),("swall_L","front_mid"),
 ("floor","wallL"),("floor","wallR"),("floor","front_low"),("floor","front_mid"),("floor","rear"),("wallL","front_low"),("wallR","front_low"),
 ("wallL","rear"),("wallR","rear"),("front_low","front_mid")]}
MOVING=["crank_hub","crank_arm","crank_boss","tip_head","pushrod","barA","b_spacer","b_rod_end","A","B","mid_head",
 "fix_clip_b","fix_head_b","fix_clip_t","fix_head_t","t_spacer","t_rod_end","tbar","plate","flangeL","flangeR"]
def run(tol=0.25,step=2.0):
    worst={}
    for th in np.arange(C.TH_DOWN,C.TH_DOWN+C.TH_SPAN+0.01,step):
        B=bodies(th)
        for m in MOVING:
            pts=B[m].sample()
            for k,o in B.items():
                if k==m or frozenset((k,m)) in JOINED: continue
                n=int(o.inside(pts,tol).sum())
                if n:
                    key=tuple(sorted((m,k)))
                    if key not in worst or n>worst[key][1]: worst[key]=(round(th,1),n)
    return worst
if __name__=="__main__":
    w=run(); print("간섭 없음 ✅" if not w else "")
    for (a,b),(th,n) in sorted(w.items()): print(f"간섭: {a} ↔ {b}  크랭크 {th}°  점 {n}")
