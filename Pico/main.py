'''
MicroProfessors
05/01/2022

Continuously listen for motor commands from Pi Zero over UART rx / tx
Use two cores on Pico to drive pairs of motors simultaneously at a rate of FREQ - e.g. North and South motor, East and West motor

Each core drives

Listen for motor commands from Pi Zero over UART rx / tx
Continuously listens on an async rx / tx channel for computed motor commands.
Will listen at a rate of FREQ and then drive motors at a rate of FREQ
'''
import math
from machine import Pin, Timer
import time
import sys
import uselect
#from listen_from_pi import *

# motors noted as follows
# oriented top = away from control
# x1 = BL = driver 1
# x2 = TR = driver 4

# y1 = BR = driver 2
# y2 = TL = driver 3

DIR_x1_GPIO = 8
DIR_x2_GPIO = 11
DIR_y1_GPIO = 9
DIR_y2_GPIO = 10

PULSE_x1_GPIO = 12
PULSE_x2_GPIO = 15
PULSE_y1_GPIO = 13
PULSE_y2_GPIO = 14

# Declare pins as output
DIR_x1 = Pin(DIR_x1_GPIO, Pin.OUT)
DIR_x2 = Pin(DIR_x2_GPIO, Pin.OUT)
DIR_y1 = Pin(DIR_y1_GPIO, Pin.OUT)
DIR_y2 = Pin(DIR_y2_GPIO, Pin.OUT)

PUL_x1 = Pin(PULSE_x1_GPIO, Pin.OUT)
PUL_x2 = Pin(PULSE_x2_GPIO, Pin.OUT)
PUL_y1 = Pin(PULSE_y1_GPIO, Pin.OUT)
PUL_y2 = Pin(PULSE_y2_GPIO, Pin.OUT)

CCW = 1     # 'up' on positive motors (x1,y1)
CW = 0      # 'down' on positive motors (x2,y2)


class stepper_controller():
    def __init__(self,pulse_frac = 1):      # pulse_frac is denom of DIP setting (1/16-->16)
        self.pulse_frac = pulse_frac
        self.motor_pos_steps = [[0,0],[0,0]]    # x1 x2 y1 y2
        self.x_target = 0
        self.y_target = 0
        self.x_step_target = 0
        self.y_step_target = 0
        self.xpins = [[DIR_x1, PUL_x1],[DIR_x2, PUL_x2]]
        self.ypins = [[DIR_y1, PUL_y1],[DIR_y2, PUL_y2]]
        self.m_theta_rng = [-7,7]
        self.m_pulse_rng = [-self.m_theta_rng[0]*200*pulse_frac,self.m_theta_rng[1]*200*pulse_frac]     # 200 full steps on motor
        self.Lp = 315/1000
        self.La = 80/1000
        self.Lc = 80/1000
        self.hm = 0.030
        self.h0 = 0.150
        self.bp = self.h0 - self.hm
        self.zero_angle_deg = 48.5904
        self.HIGH_FREQ = 400            # refers to pulse frequency
        self.uart = machine.UART(0, 115200, timeout = 200)
        self.dt_pulse = 1/5000
        self.x = 0
        self.y = 0
        self.good_x_t = 0
        self.good_y_t = 0

        self.start()
#        self.circle_jog()
#        self.ss_jog()

    def circle_jog(self, nrot=4, freq=360*2):
        '''
        Parametrized circular state-space jog. Platform normal traces a right
        circular cone.
        '''
        Timer().init(freq=self.HIGH_FREQ, mode = Timer.PERIODIC, callback = self.mot_ctrl_callback)
        th_max = math.radians(5)
        self.set_target(0,0)
        self.jog_wait()

        x = lambda t: th_max*math.sin((t/freq)*math.pi*2)
        y = lambda t: th_max*math.cos((t/freq)*math.pi*2)
        for t in range(0,freq*nrot):
            self.set_target(x(t),y(t))
            self.jog_wait()
        self.set_target(0,0)
        self.jog_wait()



    def ss_jog(self):
        '''
        Several jogs around the (+/- 5 deg) state space.
        '''
        Timer().init(freq=self.HIGH_FREQ, mode = Timer.PERIODIC, callback = self.mot_ctrl_callback)
        th_max = math.radians(5)
        self.set_target(0,0)
        self.jog_wait()
        for j in range(2):
            self.set_target(th_max,0)
            self.jog_wait()
            self.set_target(-th_max,0)
            self.jog_wait()
        self.set_target(0,0)
        self.jog_wait()

        nrot = 2
        freq = 32
        x = lambda t: th_max*math.sin((t/freq)*math.pi*2)
        y = lambda t: th_max*math.cos((t/freq)*math.pi*2)
        for t in range(0,freq*nrot):
            self.set_target(x(t),y(t))
            self.jog_wait()

        self.set_target(0,0)
        self.jog_wait()
        for j in range(2):
            self.set_target(th_max,0)
            self.jog_wait()
            time.sleep(0.1)
            self.set_target(0,th_max)
            self.jog_wait()
            time.sleep(0.1)
            self.set_target(-th_max,0)
            self.jog_wait()
            time.sleep(0.1)
            self.set_target(0,-th_max)
            self.jog_wait()
            time.sleep(0.1)

        self.set_target(0,0)
        self.jog_wait()


    def jog_wait(self, tol = 1):    # wait to execute next command
        x_err, y_err = tol+1, tol+1
        while(x_err > tol or y_err > tol):
            x_err = abs(self.x_step_target - self.motor_pos_steps[0][0])
            y_err = abs(self.y_step_target - self.motor_pos_steps[1][0])

    def start(self):
        Timer().init(freq=self.HIGH_FREQ, mode = Timer.PERIODIC, callback = self.mot_ctrl_callback)
        self.set_target(0,0)
        while True:
            damp = 0.98
            data_in = self.listen_for_commands()
            if (data_in[2] != 0 and data_in[2] != -1):
                self.set_target(data_in[-3], data_in[-2])
                self.good_x_t, self.good_y_t = [self.x_target, self.y_target]
                data_out = str(data_in[0]) + "," + str(data_in[1]) +"," + str(data_in[2]) + "," +  str(data_in[3]) + "," + str(data_in[4]) + "," + str(data_in[5])
                print(data_out)
            else:
                self.set_target(self.x_target*damp,self.y_target*damp)  # decay commanded angles on none or bad data
            time.sleep(0.01)


    def set_target(self,rad_X,rad_y):   # targets stored in rads
        self.x_target = rad_x
        [dtemp, self.x_step_target] = self.angle_in_out(rad_x)
        self.y_target = rad_y
        [dtemp, self.y_step_target] = self.angle_in_out(rad_y)


    def listen_for_commands(self):
        b = None
        msg = ""

        if self.uart.any() != 0:
            b = self.uart.readline()
            msg = str(b.decode('utf-8'))
            msg = msg.split(',')
            x, y, r, th, ph, dt = [float(k) for k in msg]
            self.x, self.y = [x,y]
            self.dt = dt
            self.r = r
            return (x, y, r, th, ph, dt)
        else:
            return (-1,-1,-1,-1,-1,-1)


    def mot_ctrl_callback(self, timer):
        x_err = self.x_step_target - self.motor_pos_steps[0][0]
        y_err = self.y_step_target - self.motor_pos_steps[1][0]

        if x_err > 0:
            DIR_x1.value(CW)
            DIR_x2.value(CCW)
        else:
            DIR_x1.value(CCW)
            DIR_x2.value(CW)

        if y_err > 0:
            DIR_y1.value(CCW)
            DIR_y2.value(CW)
        else:
            DIR_y1.value(CW)
            DIR_y2.value(CCW)

        if x_err:
            PUL_x1.value(1)
            PUL_x2.value(1)
            self.motor_pos_steps[0][0] -= pow(-1,(x_err>0)) # update step record
        if y_err:
            PUL_y1.value(1)
            PUL_y2.value(1)
            self.motor_pos_steps[1][0] -= pow(-1,(y_err>0))

        if x_err or y_err:              # pulse all 4 motors at once as needed
            time.sleep(self.dt_pulse)
            PUL_x1.value(0)
            PUL_x2.value(0)
            PUL_y1.value(0)
            PUL_y2.value(0)


    def angle_in_out(self, t_in):
        '''
        IK function for arm endpoint in rz plane
        returns deg angle and # pulses from 0 to that angle (not steps)
        '''
        r = self.Lp/2
        b = pow(pow(r*math.cos(-t_in)-r,2) + pow(r*math.sin(-t_in)-self.bp,2),1/2)
        t_mot = math.pi - math.acos(pow(b,2)/(2*self.La*b)) - math.acos( (pow(r,2)+pow(b,2)-pow(r*math.cos(t_in),2)-pow(r*math.sin(t_in)+self.bp,2))/(2*r*b) )
        mot_deg = math.degrees(t_mot) - self.zero_angle_deg
        step_ind = round((mot_deg/360)*200*self.pulse_frac)
        return mot_deg, step_ind  # relative to init


a = stepper_controller(pulse_frac = 32)





















'''
    def x_ctrl_callback(self, timer):
        mt_deg,mt_step = self.angle_in_out(self.x_target)
        mt_deg_op,mt_step_op = self.angle_in_out(-1*self.x_target)
        step_error = round(mt_step) - self.motor_pos_steps[0][0]
        step_error_op = round(mt_step_op) - self.motor_pos_steps[0][1]


        if step_error > 0:
            DIR_x1.value(CW)
            DIR_x2.value(CCW)
        else:
            DIR_x1.value(CCW)
            DIR_x2.value(CW)

        if abs(step_error) > 0:
            #print(step_error)
            PUL_x1.value(1)
            PUL_x2.value(1)
            time.sleep(0.1/1000)
            PUL_x1.value(0)
            PUL_x2.value(0)
            self.motor_pos_steps[0][0] -= pow(-1,(step_error>0))
            self.motor_pos_steps[0][1] += pow(-1,(step_error>0))

        self.y_ctrl_callback(0)

    def y_ctrl_callback(self, timer):

        mt_deg,mt_step = self.angle_in_out(self.y_target)
        mt_deg_op,mt_step_op = self.angle_in_out(-1*self.y_target)
        step_error = mt_step - self.motor_pos_steps[1][0]
        step_error_op = mt_step_op - self.motor_pos_steps[1][1]

        if step_error > 0:
            DIR_y1.value(CCW)
            DIR_y2.value(CW)
        else:
            DIR_y1.value(CW)
            DIR_y2.value(CCW)

        if abs(step_error) > 0:
            #print(step_error)
            PUL_y1.value(1)
            PUL_y2.value(1)
            time.sleep(0.1/1000)
            PUL_y1.value(0)
            PUL_y2.value(0)
            self.motor_pos_steps[1][0] -= pow(-1,(step_error>0))
            self.motor_pos_steps[1][1] += pow(-1,(step_error>0))
'''
