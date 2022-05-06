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

from send_from_pi import update_commands


class closed_loop_ctrl:
    def __init__(self):
        self.cam = pc.PiCamera()
        self.cam.framerate = 30
        self.res = [320,240]
        self.cam.resolution = (self.res[0],self.res[1])
        self.deg_rng = 10   #symmetric (value for one side)
        self.Lp = 0.160     #platform sidelength

    def runout(self):
        r = self.Lp/0.8
        period = 5
        xt = lambda t: r*math.cos(2*math.pi*(t/period))
        yt = lambda t: r*math.sin(2*math.pi*(t/period))
        x0,y0 = [0,0]
        tend = 2*period
        t0 = time.time()
        t = 0
        x = np.array([[x0,y0]])
        times = []
        e_sum = 0
        while t < tend:
            tp = t
            t = time.time()-t0
            times = times + [t]
            dt = t-tp
            xb,yb,rb = self.process_frame(self.get_frame())
            v = np.subtract([xb,yb],x[-1,:])/dt
            x = np.append(x,[[xb,yb]],0)
            e_sum = e_sum + np.subtract([xt,yt],[xb,yb])
            thout,phout = self.controller([xb,yb],[xt(t),yt(t)],v,e_sum)
            self.update_command(thout,phout)

        return x,times


    def update_command(self,t,p,dt=1/30):
        update_commands(t,p,dt)


    def controller(self,x,xt,v,e_sum=[0,0],K=[6,0,-1]):
        '''
        symmetric PID controllers in 2D. Default to no integration of error
        '''
        Kp,Ki,Kd = K
        trng = [math.radians(-1*self.deg_rng),math.radians(self.deg_rng)]
        g = 9.802
        dir_t = np.subtract(xt,x)               # vector TO target
        xy_dir = math.atan2(dir_t[1],dir_t[0])  # atan2 preserves quadrant
#        xy_dir = xy_dir+math.pi*2 % math.pi*2
        a_des = [Kp*err + Ki*int + Kd*vel for err,int,vel in zip(dir_t,v,e_sum)]
        arng = [g*math.sin(trng[0]),g*math.sin(trng[1])]
        ax = min(max(a_des[0],arng[0]),arng[1])
        ay = min(max(a_des[1],arng[0]),arng[1])

        thout = math.asin(ax/g)     # plant
        phout = math.asin(ay/g)
        return thout,phout          # absolute plate angles (theta --> x accel, phi --> y accel)

    def get_frame(self, prev=False):
        cf = np.empty((self.res[0],self.res[1],3), dtype=np.uint8)    # 24 bit depth
        self.cam.capture(cf,'rgb')
        if prev:
            cv.imshow('frame',cf)
            time.sleep(1)
            cv.destroyAllWindows()
        return cf

    def process_frame(self, cf, K_rgb=[0,0.4,1], prev=False, ret_im=False):
        imout = cf
        pf = np.empty(shape(cf),dtype=np.uint8)
        for k in range(0,3):
            pf[:,:,k] = K_rgb[k]*cf[:,:,k]
        pf = cv.cvtColor(pf, cv.COLOR_BGR2GRAY)
        imout = np.append(imout,pf,1)
        pf = cv.convertScaleAbs(pf, alpha=3, beta=0)
        pf = cv.blur(pf, (4,4))
        imout = np.append(imout,pf,1)
        circles = cv.HoughCircles(pf, cv.HOUGH_GRADIENT, 1, 100, param1 = 50, param2 = 40, minRadius=50,maxRadius=100)
        imout = np.append(imout,pf,1)
        circles = np.uint32(np.around(circles))
        num_circ = 1
        dat = circles[0]
        if prev:
            cv.imshow('frame',pf)
            time.sleep(1)
            cv.destroyAllWindows()
        if ret_im:
            return dat,imout
        return dat
