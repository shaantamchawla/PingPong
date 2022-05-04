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
FREQ = 100 # Hz

# Define stepper motor GPIO connections:
PULSE_1_2_GPIO = 16
PULSE_3_4_GPIO = 21
DIR_1_GPIO = 17
DIR_2_GPIO = 18
DIR_3_GPIO = 19
DIR_4_GPIO = 20

# Declare pins as output
DIR_4 = Pin(DIR_4_GPIO, Pin.OUT)
DIR_3 = Pin(DIR_3_GPIO, Pin.OUT)
DIR_2 = Pin(DIR_2_GPIO, Pin.OUT)
DIR_1 = Pin(DIR_1_GPIO, Pin.OUT)

PULSE_1_2 = Pin(PULSE_1_2_GPIO, Pin.OUT) ## first pair of motors
PULSE_3_4 = Pin(PULSE_3_4_GPIO, Pin.OUT) ## second pair of motors

## Direction control
CCW = 1
CW = 0
    
# Set spin direction CW or CCW
DIR_1.value(CCW)
DIR_2.value(CW)
DIR_3.value(CCW)
DIR_4.value(CCW)

## Threading for dual core
sLock = _thread.allocate_lock()

# LED
led = Pin(25, Pin.OUT)
led.toggle()

def CoreTask():
    Timer().init(freq=FREQ, mode=Timer.PERIODIC, callback=spin_motor_3_4)
    
    while True:
        sLock.acquire()
        print('spinning..')
        led.toggle()
        time.sleep(0.005)
        print("Exiting from the 2nd Thread")
        sLock.release()

def spin_motor_1_2(timer):
    PULSE_1_2.value(1)
    PULSE_1_2.value(0)
    
def spin_motor_3_4(timer):
    PULSE_3_4.value(1)
    PULSE_3_4.value(0)

def main():
    _thread.start_new_thread(CoreTask, ())
    Timer().init(freq=FREQ, mode=Timer.PERIODIC, callback=spin_motor_1_2)
    
    while True:
        # We acquire the semaphore lock
        sLock.acquire()
        print("Entered into the main Thread. spinning...")
        led.toggle()
        time.sleep(0.005)
        print("Toggling")
        sLock.release()
     
main()