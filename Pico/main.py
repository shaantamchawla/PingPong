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
    def __init__(self,pulse_frac = 1):      # pulse_frac = denom of DIP setting (1/16 --> 16)
        self.pulse_frac = pulse_frac
        self.motor_pos_steps = [[0,0],[0,0]]    # x1 x2 y1 y2
        self.x_target = 0
        self.y_target = 0
        self.xpins = [[DIR_x1, PUL_x1],[DIR_x2, PUL_x2]]
        self.ypins = [[DIR_y1, PUL_y1],[DIR_y2, PUL_y2]]
        self.m_theta_rng = [-10,10]
        self.m_pulse_rng = [-10*pulse_frac,10*pulse_frac]
        self.Lp = 315/1000
        self.La = 80/1000
        self.Lc = 80/1000
        self.hm = 0.030
        self.h0 = 0.150
        self.bp = self.h0 - self.hm
        self.zero_angle_deg = 48.5904
        self.HIGH_FREQ = 50

        self.start()

    def start(self):
        Timer().init(freq=self.HIGH_FREQ, mode = Timer.PERIODIC, callback = self.x_ctrl_callback)
        self.set_target(0,6)
        self.set_target(1,6)
        while True:
            time.sleep(0.001)

    def set_target(self,axis,deg):  # x --> 0    y --> 1
        if axis == 0:
            self.x_target = deg
        if axis == 1:
            self.y_target = deg


    def x_ctrl_callback(self, timer):

        mt_deg,mt_step = self.angle_in_out(self.x_target)
        print(mt_deg, mt_step)
        mt_deg_op,mt_step_op = self.angle_in_out(-1*self.x_target)
        step_error = mt_step - self.motor_pos_steps[0][0]
        step_error_op = mt_step_op - self.motor_pos_steps[0][1]
        

        print(mt_step,self.motor_pos_steps[0][0],step_error)
        
        if step_error > 0:
            DIR_x1.value(CW)
            DIR_x2.value(CCW)
        else:
            DIR_x1.value(CCW)
            DIR_x2.value(CW)

        if abs(step_error) > 0:
            PUL_x1.value(1)
            PUL_x2.value(1)
            time.sleep(0.001)
            PUL_x1.value(0)
            PUL_x2.value(0)
            self.motor_pos_steps[0][0] += 1*(step_error/abs(step_error))
            self.motor_pos_steps[0][1] -= 1*(step_error/abs(step_error))
            
        self.y_ctrl_callback(0)

    def y_ctrl_callback(self, timer):

        mt_deg,mt_step = self.angle_in_out(self.y_target)
        mt_deg_op,mt_step_op = self.angle_in_out(-1*self.y_target)
        step_error = mt_step - self.motor_pos_steps[1][0]
        step_error_op = mt_step_op - self.motor_pos_steps[1][1]
        

        print(mt_step,self.motor_pos_steps[1][0],step_error)
        
        if step_error > 0:
            DIR_y1.value(CW)
            DIR_y2.value(CCW)
        else:
            DIR_y1.value(CCW)
            DIR_y2.value(CW)

        if abs(step_error) > 0:
            PUL_y1.value(1)
            PUL_y2.value(1)
            time.sleep(0.001)
            PUL_y1.value(0)
            PUL_y2.value(0)
            self.motor_pos_steps[1][0] += 1*(step_error/abs(step_error))
            self.motor_pos_steps[1][1] -= 1*(step_error/abs(step_error))            


    def angle_in_out(self, plat_deg):
        r = self.Lp/2

        t_in = math.radians(plat_deg)
        b = pow(pow(r*math.cos(-t_in)-r,2) + pow(r*math.sin(-t_in)-self.bp,2),1/2)
        t_mot = math.pi - math.acos((pow(self.La,2)-pow(self.Lc,2)+pow(b,2))/(2*self.La*b)) - math.acos((pow(r,2)+pow(b,2)-pow(r*math.cos(t_in),2)-pow(r*math.sin(t_in)+self.bp,2))/(2*r*b))
        mot_deg = math.degrees(t_mot)
        mot_deg = mot_deg - self.zero_angle_deg
        step_ind = round((mot_deg/360)*200*self.pulse_frac)
        return mot_deg, step_ind  # relative to init
    
    
a = stepper_controller(pulse_frac = 2)
a.start()
