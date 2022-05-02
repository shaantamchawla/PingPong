'''
MicroProfessors
04/26/2022

Continuous rotation script for 1 motor
'''

from machine import Pin, Timer
import time

# Step info
step_delay = 0.001 # in seconds - min delay allowable at full step

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
DIR.value(CCW)

def main():
    led = Pin(25, Pin.OUT)
    led.toggle()
    
    while True:
        PULSE.value(1)
        time.sleep(step_delay)
        PULSE.value(0)
        time.sleep(step_delay)
        
main()