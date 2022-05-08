'''
MicroProfessors
04/30/2022

Send motor commands from Pi Zero -> Pico over UART
Sends a tuple of the form (float1, float2) that receiver can decode and use to drive motors
Happens asynchronously; if there is delay, pico patiently waits and then acts on commands once they arrive.

Wiring: TX on PiZero -> RX on Pico. RX on PiZero -> TX on Pico.
'''

import time
import serial

FREQ = 30 # Hz
i = 0

def send_commands(x,y,r,th,ph,dt):
    ser = serial.Serial(
  port='/dev/serial0', # Change this according to connection methods, e.g. /dev/ttyUSB0
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
    )
    [x,y,r,th,ph,dt] = [round(k,4) for k in [x,y,r,th,ph,dt]]
    #print(x,y,r,th,ph,dt)
## TODO - get rid of spoofed data
    dat = str(x)+','+str(y)+','+str(r)+','+str(th)+','+str(ph)+','+str(dt)+'\n'
    print(x,y)
    print(th,ph)
    print()
    ser.write(dat.encode('utf-8'))
#    print("Send message {}".format(i))
#    time.sleep(1.0 / FREQ)
