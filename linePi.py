import RPi.GPIO as IO          #calling header file which helps us use GPIO’s of PI

import time                            #calling time to provide delays in program

IO.setwarnings(False)           #do not show any warnings

IO.setmode (IO.BCM)


IO.setup(19,IO.OUT)           # initialize GPIO19 as an output.
IO.setup(20,IO.OUT)
IO.setup(21,IO.OUT)
IO.setup(22,IO.OUT)
IO.setup(23,IO.OUT)

au = IO.PWM(19,1000)          #GPIO19 as PWM output, with 1000Hz frequency
au.start(0) 

ai = IO.PWM(20,1000)          #GPIO19 as PWM output, with 1000Hz frequency
ai.start(0)

ru = IO.PWM(21,1000)          #GPIO19 as PWM output, with 1000Hz frequency
ru.start(0)

th = IO.PWM(22,1000)          #GPIO19 as PWM output, with 1000Hz frequency
th.start(0)

el = IO.PWM(23,1000)          #GPIO19 as PWM output, with 1000Hz frequency
el.start(0)
def aux(x):
    au.ChangeDutyCucle(x)

def ail(x):
    ai.ChangeDutyCucle(x)

def rud(x):
    ru.ChangeDutyCucle(x)

def thr(x):
    th.ChangeDutyCucle(x)

def ele(x):
    el.ChangeDutyCucle(x)

                             #generate PWM signal with 0% duty cycle

while 1:                               #execute loop forever

    for x in range (50):                          #execute loop for 50 times, x being incremented from 0 to 49.
        p.ChangeDutyCycle(x)               #change duty cycle for varying the brightness of LED.
        time.sleep(0.1)                           #sleep for 100m second
      
    for x in range (50):                         #execute loop for 50 times, x being incremented from 0 to 49.
        p.ChangeDutyCycle(50-x)        #change duty cycle for changing the brightness of LED.
        time.sleep(0.1)


while 1:
    
