## send state information from pico -> GUI over usb serial
## write motor commands to pico

from machine import UART, Pin
import time
import sys

## Offline debugging function to ensure Pico is alive
def blink_led():
    led = Pin(25, Pin.OUT)
    led.toggle()
    time.sleep(0.1)

def listen_for_commands():
    ## see if we have anything from GUI, over usb
    # msg_usb = sys.stdin.readline()
        
    #print(msg_usb)
    
    ## default = read from pi zero
    return "-1,-2,-3,-4,-5,-6"

def send_state_to_serial():
    data_out = listen_for_commands()
    ## listen, set targets, then print to serial
    
    print(data_out)

while True:
    #blink_led()
    send_state_to_serial()
    time.sleep(0.1)
    