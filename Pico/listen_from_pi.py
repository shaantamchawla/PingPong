'''
MicroProfessors
04/28/2022

Listen for motor commands from Pi Zero over UART rx / tx
Continuously listens on an async rx / tx channel for computed motor commands.
Will listen at a rate of FREQ and then drive motors at a rate of FREQ
'''

import os
import machine
from machine import Pin, Timer
import time

FREQ = 10 # Hz
#motor_command_x, motor_command_y = None, None # Commands to listen for

# Open a UART connection
uart = machine.UART(0, 115200)

def listen_for_commands():
    b = None
    msg = ""
       
     if uart.any():
        b = uart.readline()

        try:
            msg = (str(b.decode('utf-8')))
            x, y, r, th, ph = eval(msg)
            return (x, y, r, th, ph)

#motor_command_x, motor_command_y = (eval(msg))
        except:
            pass
