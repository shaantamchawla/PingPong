'''
MicroProfessors
04/26/2022

Continuous rotation script for 1 motor PAIR

Now testing dual core for multitasking — 1 for motor drive, 1 for toggle LED
Dual core basics: https://circuitdigest.com/microcontroller-projects/dual-core-programming-on-raspberry-pi-pico-using-micropython
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
sLock = _thread.allocate_lock()

# LED
led = Pin(25, Pin.OUT)
led.toggle()

def CoreTask():
    Timer().init(freq=FREQ, mode=Timer.PERIODIC, callback=spin_motor)
    
    while True:
        sLock.acquire()
        print('spinning..')
        led.toggle()
        time.sleep(0.05)
        print("Exiting from the 2nd Thread")
        sLock.release()

def spin_motor(timer):
    PULSE.value(1)
    PULSE.value(0)

def main():
    _thread.start_new_thread(CoreTask, ())
    
    while True:
        # We acquire the semaphore lock
        sLock.acquire()
        print("Entered into the main Thread")
        led.toggle()
        time.sleep(0.05)
        print("Toggling")
        sLock.release()
     
main()