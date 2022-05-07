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

## Direction control
CCW = 1
CW = 0

# Set spin direction CW or CCW
DIR_x1.value(CCW)
DIR_x2.value(CW)
DIR_y1.value(CCW)
DIR_y2.value(CW)

## Threading for dual core
#sLock = _thread.allocate_lock()

# LED
led = Pin(25, Pin.OUT)
led.toggle()

'''
def CoreTask():
    Timer().init(freq=FREQ, mode=Timer.PERIODIC, callback=spin_motor_y)

    while True:
        sLock.acquire()
        print('spinning..')
        led.toggle()
        time.sleep(0.005)
        print("Exiting from the 2nd Thread")
        sLock.release()
'''

def spin_motor_x(timer):
    PUL_x1.value(1)
    PUL_x2.value(1)
    PUL_x1.value(0)
    PUL_x2.value(0)


def spin_motor_y(timer):
    PUL_y1.value(1)
    PUL_y2.value(1)
    PUL_y1.value(0)
    PUL_y2.value(0)


def main():
    #_thread.start_new_thread(CoreTask, ())
    Timer().init(freq=FREQ, mode=Timer.PERIODIC, callback=spin_motor_x)

    while True:
        # We acquire the semaphore lock
        #sLock.acquire()
        print("Entered into the main Thread. spinning...")
        led.toggle()
        time.sleep(0.005)
        print("Toggling")
        #sLock.release()

main()
