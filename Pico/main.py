'''
MicroProfessors
05/01/2022

Continuously listen for motor commands from Pi Zero over UART rx / tx
Use two cores on Pico to drive pairs of motors simultaneously at a rate of FREQ - e.g. North and South motor, East and West motor

Each core drives

Listen for motor commands from Pi Zero over UART rx / tx
Continuously listens on an async rx / tx channel for computed motor commands.
Will listen at a rate of FREQ and then drive motors at a rate of FREQ
'''