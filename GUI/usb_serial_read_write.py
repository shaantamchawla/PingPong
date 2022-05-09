## send state information from pico -> GUI over usb serial
## write motor commands to pico

from machine import UART, Pin
import time
import sys, uselect

## Offline debugging function to ensure Pico is alive
def blink_led():
    led = Pin(25, Pin.OUT)
    led.toggle()
    time.sleep(0.1)

def listen_for_commands():
    buff = []
    
    ## see if we have anything from GUI, over usb
    file = open('log.txt', 'a')
    select_result = uselect.select([sys.stdin], [], [], 0)
    while select_result[0]:
        input_character = sys.stdin.read(1)
        buff.append(input_character)
        select_result = uselect.select([sys.stdin], [], [], 0)
        
    if '\n' in buff:
        blink_led()
        end_index = buff.index('\n')
        input_line = "".join(buff[:end_index])
        return str(input_line)
        
        if end_index < len(buff):
            buff = buff[end_index + 1 :]
        else:
            buff = []
    else:
        input_line = ""
    
    ## default = read from pi zero
    return "10,12,13,14,15,16"

def send_state_to_serial():
    data_out = listen_for_commands()
    ## listen, set targets, then print to serial
    
    print(data_out)

while True:
    #blink_led()
    send_state_to_serial()
    time.sleep(0.1)
    