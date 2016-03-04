# Argument 1 is pin to output pwm to 
# Argument 2 is value to output to pwm (0-1)

import sys
import mraa
import time

pin = sys.argv[1]
value = sys.argv[2]

x = mraa.Pwm(int(pin))
x.period_ms(23)
x.enable(True)

x.write(float(value))

while True:
    y = 1
    
    
