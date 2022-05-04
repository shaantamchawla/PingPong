'''
MicroProfessors
04/26/2022

Continuous rotation script for 1 motor PAIR
'''

from machine import Pin, Timer
import time
import _thread

# Step info
FREQ = 100 # Hz

# Define stepper motor GPIO connections:
DIR_2_GPIO = 18
DIR_1_GPIO = 17
PULSE_GPIO = 16

# Declare pins as output
DIR_2 = Pin(DIR_2_GPIO, Pin.OUT)
DIR_1 = Pin(DIR_1_GPIO, Pin.OUT)
PULSE = Pin(PULSE_GPIO, Pin.OUT)

## Direction control
CCW = 1
CW = 0
    
# Set spin direction CW or CCW
DIR_1.value(CCW)
DIR_2.value(CW)

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
