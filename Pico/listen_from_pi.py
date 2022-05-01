'''
MicroProfessors
04/28/2022

Listen for motor commands from Pi Zero
'''

import os
import machine
from machine import Pin, Timer
import time

# Open a UART connection
uart = machine.UART(0, 115200)
print(uart)

b = None
msg = ""
while True:
    time.sleep(1)
    if uart.any():
        b = uart.readline()
        print(type(b))
        print(b)
        try:
            msg = b.decode('utf-8')
            print(type(msg))
            print(">> " + msg)
        except:
            pass