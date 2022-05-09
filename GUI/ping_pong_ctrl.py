import numpy as np
import math
from matplotlib import pyplot as plt
import time
import os
import imageio


def controller(x,v,xt):         # length units of m
    Kp = 6
    Kd = -1


    trng = [math.radians(-15),math.radians(15)]
    g = 9.802
    dir_t = np.subtract(xt,x)
    xy_dir = math.atan2(dir_t[1],dir_t[0])  # atan2 preserves quadrant      represents angle TOWARDS TARGET
    xy_dir = xy_dir+math.pi*2 % math.pi*2
    a_des = [(Kp*dir_t[0] + Kd*v[0]), (Kp*dir_t[1] + Kd*v[1])]   # xydir + 180 (in rad, rh up tangent)
    rng = [g*math.sin(trng[0]),g*math.sin(trng[1])]
    ax = max(a_des[0],rng[0])
    ax = min(ax,rng[1])   #saturation
    ay = min(max(a_des[1],rng[0]),rng[1])

    thout = math.asin(ax/g)    # plant
    phout = math.asin(ay/g)
    return thout,phout          # absolute state angles


def rotate(v, theta):      #theta in deg
    theta = math.radians(theta)
    rt = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return np.matmul(rt, v)


tdat = []
xdat = []
ydat = []
axdat = []
aydat = []
x0,y0 = [0,0]
xc,yc = [x0,y0]
xt,yt = [0,0]
vxc, vyc = [0,0]
g = 9.802
tend = 10
freq = 1000
npoints = tend*freq
dt = tend/npoints
fig, ax = plt.subplots()
tol = 4/1000
ltrail = freq
L = 0.16
r = (L/2)/2
n = 1
filenames = []

for t in np.linspace(0,tend,npoints):
    xt,yt = [r*math.cos((n*t/tend)*2*math.pi), r*math.sin((n*t/tend)*2*math.pi)]
    d_curr = controller([xc,yc],[vxc,vyc],[xt,yt])
    vxc = vxc + dt*g*math.sin(d_curr[0])
    vyc = vyc + dt*g*math.sin(d_curr[1])
    yc = yc + vyc*dt
    xc = xc + vxc*dt
    tdat = tdat + [t]
    xdat = xdat + [xc]
    ydat = ydat + [yc]
    axdat = axdat + [dt*g*math.sin(d_curr[0])]
    aydat = aydat + [dt*g*math.sin(d_curr[1])]
    plt.gca().set_aspect('equal', adjustable='box')
    ax.set_xlim(-1.2*(L/2),1.2*(L/2))
    ax.set_ylim(-1.2*(L/2),1.2*(L/2))
    ax.plot(x0,y0,marker='o')
    ax.plot(xt,yt,marker='s')
    ax.plot([-L/2,L/2,L/2,-L/2,-L/2],[-L/2,-L/2,L/2,L/2,-L/2])
    if np.linalg.norm([xc-xt,yc-yt]) < tol:
        ax.plot(xc,yc,'g+',markersize=20)
        ax.plot(xc,yc,'gx',markersize=20)
        ax.plot(xdat[-ltrail:-1],ydat[-ltrail:-1],'r+')
    elif len(xdat)<ltrail:
        ax.plot(xdat,ydat,'r+')
    else:
        ax.plot(xdat[-ltrail:-1],ydat[-ltrail:-1],'r+')
    ax.plot([xt,xc],[yt,yc],ls=':')

    fnm = f'{t}.png'
    filenames.append(fnm)
    plt.savefig(fnm)

    plt.show(block=False)
    plt.pause(0.0001)
    plt.cla()

with imageio.get_writer('follow_circle.gif', mode='I') as writer:
    for filename in filenames:
        image = imageio.imread(filename)
        writer.append_data(image)

# Remove files
for filename in filenames:
    os.remove(filename)
plt.show(block=False)
plt.pause(2)
plt.close('all')
