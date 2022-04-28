## Periodically fetch image
## Run circle detect code on it
## Multi tasking: while in one thread we are fetching image at 30 hz, in other thread
	## we are using circle detect -> determine height / x / y -> compute motor control commands
	## then, send motor command over UART to pico
	## pico takes care of motor driving
