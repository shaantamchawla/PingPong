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



DIR_x1_GPIO = 18
DIR_x2_GPIO = 17
PULSE_x1_GPIO = 16
PULSE_x2_GPIO = 15  # UPDATE PINS

DIR_y1_GPIO = 18    # UPDATE PINS
DIR_y2_GPIO = 17    # UPDATE PINS
PULSE_y1_GPIO = 16  # UPDATE PINS
PULSE_y2_GPIO = 15  # UPDATE PINS

class stepper_controller():
    def __init__(self,pulse_frac = 1):      # pulse_frac = denom of DIP setting (1/16 --> 16)
        self.pulse_frac = pulse_frac
        self.motor_pos_steps = [0,0,0,0]
        self.xpins = np.array([[DIR_x1_GPIO,DIR_x2_GPIO],[PULSE_x1_GPIO,PULSE_x2_GPIO]])
        self.ypins = np.array([[DIR_y1_GPIO,DIR_y2_GPIO],[PULSE_y1_GPIO,PULSE_y2_GPIO]])
