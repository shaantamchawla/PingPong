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
        self.res = [960,720]
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
            e_sum = e_sum + np.subtract([xt(t),yt(t)],[xb,yb])
            thout,phout = self.controller([xb,yb],[xt(t),yt(t)],v,e_sum)
            self.update_command(xb, yb, rb, thout,phout)

        return x,times

    ## calls function in send_from_pi to send stuff to pico
    def update_command(self,x, y, r, t,p,dt=1/30):
        send_commands(x, y, r, t,p,dt)


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
        cf = np.empty((self.res[1],self.res[0],3), dtype=np.uint8)    # 24 bit depth
        self.cam.capture(cf,'bgr')
        cv.imwrite('cf.jpg', cf)
        if prev:
            cv.imshow('frame',cf)
            time.sleep(1)
            cv.destroyAllWindows()
        return cf

    def process_frame(self, cf, K_rgb=[1,0.4,0], prev=False, ret_im=False, min_radius=80, max_radius=150):
        imout = cf
        pf = np.empty(np.shape(cf),dtype=np.uint8)

        for k in range(0,3):
            pf[:,:,k] = K_rgb[2-k]*cf[:,:,k]
        pf = cv.cvtColor(pf, cv.COLOR_BGR2GRAY)
        print(pf.shape)
        #imout = np.append(imout,np.array([[pf], [np.zeros(pf.shape)], [np.zeros(pf.shape)]]), 0)
        pf = cv.convertScaleAbs(pf, alpha=8, beta=0)
        print(pf.shape)
        pf = cv.blur(pf, (4,4))
        #imout = np.append(imout,pf,1)
        circles = cv.HoughCircles(pf, cv.HOUGH_GRADIENT, 1, 100, param1 = 50, param2 = 40, minRadius=min_radius,maxRadius=max_radius)
        print(circles)

#        cv.circle(pf, (circles[0], circle[1]

        if circles is not None:
            circles = np.uint32(np.around(circles))
            num_circ = 1
            dat = circles[0][0]

            cv.circle(pf, (dat[0], dat[1]), dat[2], (0,255,0),2)
            cv.imwrite('pf.jpg', pf)
        else:
            dat = [0,0,0]
        cv.imwrite('pf.jpg', pf)
        #imout = np.append(imout,pf,1)

        if prev:
            cv.imshow('frame',pf)
            time.sleep(1)
            cv.destroyAllWindows()
        if ret_im:
            return dat,imout
        return dat
