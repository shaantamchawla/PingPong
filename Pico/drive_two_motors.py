'''
MicroProfessors
04/26/2022

Continuous rotation script for 1 motor PAIR
'''

from machine import Pin, Timer
import time
import _thread

# Step info
FREQ = 50 # Hz

# Define stepper motor GPIO connections:
DIR_GPIO = 17
PULSE_GPIO = 16

# Declare pins as output
DIR = Pin(DIR_GPIO, Pin.OUT)
PULSE = Pin(PULSE_GPIO, Pin.OUT)

## Direction control
CCW = 1
CW = 0
    
# Set spin direction CW or CCW
DIR.value(CW)

## Threading for dual core
#sLock = _thread.allocate_lock()

# LED
led = Pin(25, Pin.OUT)
led.toggle()

def spin_motor(timer):
    PULSE.value(1)
    PULSE.value(0)

def main():
    Timer().init(freq=FREQ, mode=Timer.PERIODIC, callback=spin_motor)
     
main()
