import math, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import gen as G
from meshlib import Part, extrude, rect, rot_z

def arm_tf(p0, p1, y_outer, sign):
    d = np.array([p1[0]-p0[0], 0, p1[1]-p0[1]]); d /= np.linalg.norm(d)
    ex, ey, ez = d, np.array([-d[2], 0, d[0]]), np.array([0, -1.0, 0])
    R = np.stack([ex, ey, ez], 1)
    if sign < 0:  # mirror side: thickness toward +y
        R = R @ np.diag([1, 1, -1]); 
    t = np.array([p0[0], sign * y_outer, p0[1]])
    return R, t

def assembly(a):
    xr = G.XF + G.x_rel(a); zt = G.ZP + G.h_of(a)
    tip = np.array([G.XS - G.HORN_R*math.cos(math.radians(a)), G.HORN_R*math.sin(math.radians(a))])
    tab = np.array([xr - G.DTAB, 0])
    items = []
    items.append((G.make_base(), np.eye(3), (0,0,0), "#3a3a3a"))
    items.append((G.make_pi_tray(), np.eye(3), (0,0,0), "#555555"))
    items.append((G.make_top_plate(), np.eye(3), (0,0,zt), "#f07c1e"))
    arm = G.make_arm()
    for sign in (1, -1):
        R, t = arm_tf((G.XF, G.ZP), (xr, zt), G.A_Y[1], sign); items.append((arm, R, t, "#222222"))
        R, t = arm_tf((xr, G.ZP), (G.XF, zt), G.B_Y[1], sign); items.append((arm, R, t, "#f07c1e"))
    bar = G.make_slider_bar()
    items.append((bar, np.eye(3), (xr, 0, G.ZP), "#888888"))
    items.append((bar, np.eye(3), (xr, 0, zt), "#888888"))
    ang = 180 - a
    items.append((G.make_crank(), rot_z(ang), (G.XS, 0, G.Z_CRANK), "#c0c0c0"))
    d = tab - tip
    items.append((G.make_pushrod(), rot_z(math.degrees(math.atan2(d[1], d[0]))), (tip[0], tip[1], G.Z_ROD0), "#d0d0d0"))
    xc = G.XS + G.SHAFT_OFF
    servo = Part("servo").add(extrude(rect(xc-20.35, -9.85, xc+20.35, 9.85), [], G.Z_EAR-26.6, G.Z_EAR+10.6))
    servo.add(extrude(rect(xc-27.25, -9.85, xc+27.25, 9.85), [], G.Z_EAR, G.Z_EAR+2.5))
    items.append((servo, np.eye(3), (0,0,0), "#1f4fa0"))
    # Pi board
    pi = Part("pi").add(extrude(rect(-66, -42.5, -10, 42.5), [], 8.5, 10.1))
    items.append((pi, np.eye(3), (0,0,0), "#1e8a3c"))
    return items

from matplotlib.colors import to_rgb
def draw(ax, a, title, skip=()):
    Ts, Cs = [], []
    light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
    for part, R, t, col in assembly(a):
        if part.name in skip: continue
        T = part.transformed(R, t).tris()
        n = np.cross(T[:,1]-T[:,0], T[:,2]-T[:,0]); n /= np.linalg.norm(n, axis=1)[:,None] + 1e-12
        sh = 0.45 + 0.55*np.clip(np.abs(n @ light), 0, 1)
        Ts.append(T); Cs.append(np.clip(np.array(to_rgb(col))[None,:]*sh[:,None], 0, 1))
    pc = Poly3DCollection(np.concatenate(Ts), facecolors=np.concatenate(Cs), linewidths=0)
    ax.add_collection3d(pc)
    ax.set_xlim(-75, 165); ax.set_ylim(-120, 120); ax.set_zlim(0, 240)
    ax.set_box_aspect((240, 240, 240))
    ax.view_init(elev=18, azim=-58); ax.set_title(title, fontsize=13)
    ax.set_axis_off()


fig = plt.figure(figsize=(16, 13), dpi=100)
specs = [(G.ANG_DOWN, "DOWN (servo 172 deg)", (), 22, -60), (G.ANG_UP, "UP +100 mm (servo 32 deg)", (), 22, -60),
         (G.ANG_DOWN, "mechanism, top plate hidden - DOWN", ("top_plate",), 55, -110),
         (G.ANG_UP, "mechanism, top plate hidden - UP", ("top_plate",), 55, -110)]
for i, (a, t, skip, el, az) in enumerate(specs):
    ax = fig.add_subplot(2, 2, i+1, projection="3d"); draw(ax, a, t, skip); ax.view_init(elev=el, azim=az)
plt.subplots_adjust(left=0, right=1, top=0.96, bottom=0, wspace=0, hspace=0.05)
plt.savefig("preview.png")
