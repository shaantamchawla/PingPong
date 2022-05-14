## Periodically fetch image
## Run circle detect code on it
## Multi tasking: while in one thread we are fetching image at 30 hz, in other thread
	## we are using circle detect -> determine height / x / y -> compute motor control commands
	## then, send motor command over UART to pico
	## pico takes care of motor driving

import cv2 as cv
import numpy as np
import picamera as pc
import time
import os
import math

from send_from_pi import send_commands


class closed_loop_ctrl:
    def __init__(self):
        self.cam = pc.PiCamera()
        self.cam.framerate = 30
        self.res = [256,240]
        self.cam.resolution = (self.res[0],self.res[1])
        self.deg_rng = 10   #symmetric (value for one side)
        self.Lp = 0.160     #platform sidelength
        self.Kpc = 0.160/(144*self.res[0]/155)
        self.ballpx = round((50/480)*self.res[0])

    def runout(self, tend=10):
        '''
        Implements closed loop control. Gets and processes frame, yielding (x,y,r)
        packet. Estimates velocity states, as well as integrates error. Implements
        PID control law with symmetric coefficients and outputs control values to
        raspi Pico.


        Parametrized circular target sequence
        r = self.Lp/0.8
        T = 5
        xt = lambda t: r*math.cos(2*math.pi*(t/T))
        yt = lambda t: r*math.sin(2*math.pi*(t/T))
        '''
        xt = lambda t: 0    # target platform center
        yt = lambda t: 0

        x0,y0 = [0,0]
        x = np.array([[x0,y0]])
        t = 0
        e_sum = 0
        t0 = time.time()
        while t < tend:
            tp = t
            t = time.time()-t0
            dt = t-tp
            xb,yb,rb = self.process_frame(self.get_frame(), min_radius = self.ballpx-15, max_radius = self.ballpx+15)
            vlim = 0.015    # mitigate noisy differentiation
            vx = min(vlim, max(np.subtract(xb,x[-1,0])/dt, -vlim))
            vy = min(vlim, max((yb-x[-1,1])/dt, -vlim))
            x = np.append(x,[[xb,yb]],0)
            e_sum = e_sum + np.subtract([xt(t),yt(t)],[xb,yb])
            thout,phout = self.controller([xb,yb],[xt(t),yt(t)],[vx,vy],e_sum)
            self.update_command(xb, yb, rb, thout,phout)
        return x

    def update_command(self,x, y, r, t,p,dt=1/30):
        '''
        wrapper fn to send UART command packets (not sure why we have this)
        '''
        send_commands(x, y, r, t,p,dt)


    def controller(self,x,xt,v,e_sum=[0,0],K=[10,0,0]):
        '''
        symmetric PID controllers in 2D
        '''
        g = 9.802
        Kp,Ki,Kd = K
        trng = [math.radians(-1*self.deg_rng),math.radians(self.deg_rng)]
        arng = [g*math.sin(trng[0]),g*math.sin(trng[1])]

        dir_t = np.subtract(xt,x)               # vector TO target
        xy_dir = math.atan2(dir_t[1],dir_t[0])  # atan2 preserves quadrant
        a_des = [Kp*err + Ki*int + Kd*vel for err,int,vel in zip(dir_t,v,e_sum)]    # control law

        ax = min(max(a_des[0],arng[0]),arng[1]) # bound accel/angle
        ay = min(max(a_des[1],arng[0]),arng[1])
        thout = math.asin(ax/g)                 # plant
        phout = math.asin(ay/g)                 # plate angles in rads
        return thout,phout                      # (+th-->+ax  +ph-->+ay)


    def get_frame(self, sv=False):
        cf = np.empty((self.res[1],self.res[0],3), dtype=np.uint8)    # 24 bit depth
        self.cam.capture(cf,'bgr')
        if sv:
            cv.imwrite('./ims/raw_im.jpg', cf)
        return cf

    def process_frame(self, cf, min_radius=35, max_radius=60, sv=False):
        pf = np.empty(np.shape(cf),dtype=np.uint8)

        pf = cv.cvtColor(pf, cv.COLOR_BGR2HSV)
        pf = 255 - pf[:,:,2]

        if sv:
            cv.imwrite('./ims/inv_hsv.jpg',pf)

        circles = cv.HoughCircles(pf, cv.HOUGH_GRADIENT, 1, 100, param1 = 50, param2 = 40, minRadius=min_radius,maxRadius=max_radius)
        if circles is not None:
            circles = np.uint32(np.around(circles))
            if sv:
                cdat = circles[0][0]
                cv.circle(pf, (cdat[0], cdat[1]), cdat[2], (0,255,0),2)
                cv.imwrite('./ims/circles_found.jpg', pf)
            dat = [float(k) for k in circles[0][0]]                 # take only strongest circle
            dat[0:2] = [dat[0]-self.res[0]/2, dat[1]-self.res[1]/2]
            dat[0:2] = [dat[0]*self.Kpc, dat[1]*self.Kpc]
        else:
            dat = [0,0,0]
        if sv:
            cv.imwrite('./ims/processed_frame.jpg', pf)
        return dat


c = closed_loop_ctrl()
c.runout(t=300)
