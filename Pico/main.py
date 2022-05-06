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
import numpy as np
import math
#from listen_from_pi import *

# motors noted as follows
# oriented top = away from control
# x1 = BL = driver 1
# x2 = TR = driver 4

# y1 = BR = driver 2
# y2 = TL = driver 3

DIR_x1_GPIO = 11
DIR_x2_GPIO = 15
DIR_y1_GPIO = 12
DIR_y2_GPIO = 14

PULSE_x1_GPIO = 16
PULSE_x2_GPIO = 20
PULSE_y1_GPIO = 17
PULSE_y2_GPIO = 19

class stepper_controller():
    def __init__(self,pulse_frac = 1):      # pulse_frac = denom of DIP setting (1/16 --> 16)
        self.pulse_frac = pulse_frac
        self.motor_pos_steps = [0,0,0,0]
        self.x_target = 0
        self.y_target = 0
        self.xpins = np.array([[DIR_x1_GPIO,DIR_x2_GPIO],[PULSE_x1_GPIO,PULSE_x2_GPIO]])
        self.ypins = np.array([[DIR_y1_GPIO,DIR_y2_GPIO],[PULSE_y1_GPIO,PULSE_y2_GPIO]])
        self.m_theta_rng = [-10,15]
        self.m_pulse_rng = np.multiply(self.m_theta_rng,pulse_frac)
        self.Lp = 160/1000
        self.La = 80/1000
        self.Lc = 80/1000
        self.hm = 0.030
        self.h0 = 0.150
        self.bp = self.h0 - self.hm
        self.zero_angle_deg = 48.5904


    def set_target(self,axis,deg):  # x --> 0    y --> 1
        if axis == 0:
            self.x_target = deg
        if axis == 0:
            self.y_target = deg


    def loop(self):
        mt_deg,mt_step = self.angle_in_out(self.x_target)
        step_error = mt_step - self.motor_pos_steps[0]
        while step_error > 0:
            step
        return None

    def angle_in_out(self, plat_deg):
        r = self.Lp/2

        t_in = math.radians(plat_deg)
        b = pow(pow(r*math.cos(-t_in)-r,2) + pow(r*math.sin(-t_in)-self.bp,2),1/2)
        t_mot = math.pi - math.acos((pow(self.La,2)-pow(self.Lc,2)+pow(b,2))/(2*self.La*b)) - math.acos((pow(r,2)+pow(b,2)-pow(r*math.cos(t_in),2)-pow(r*math.sin(t_in)+self.bp,2))/(2*r*b))
        mot_deg = math.degrees(t_mot)
        mot_deg = mot_deg - self.zero_angle_deg
        step_ind = round((mot_deg/360)*200*self.pulse_frac)
        return mot_deg, step_ind  # relative to init
