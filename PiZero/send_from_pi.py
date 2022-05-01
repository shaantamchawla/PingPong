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

FREQ = 10 # Hz

ser = serial.Serial(
  port='/dev/serial0', # Change this according to connection methods, e.g. /dev/ttyUSB0
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)

## This function will get filled-in to compute some motor commands based on ball trajectory
def get_motor_commands():
    float1, float2 = 3.0, 4.0
    return float1, float2

msg = ""
i = 0
while True:
    i += 1
    float1, float2 = get_motor_commands()

    print("Send message {}".format(i))
    ser.write(('(' + str(float1) + ',' + str(float2) + ')\n').encode('utf-8'))
    time.sleep(1.0 / FREQ)
