import cv2
import numpy as np
from time import sleep
import RPi.GPIO as IO


stThrottle = 50.0
throttle = 0.0
aileron = 0.0
auxi = 0.0
elevate = 0.0
rudder = 0.0

IO.setwarnings(False)           

IO.setmode (IO.BCM)


IO.setup(19,IO.OUT)           
IO.setup(20,IO.OUT)
IO.setup(21,IO.OUT)
IO.setup(22,IO.OUT)
IO.setup(23,IO.OUT)

au = IO.PWM(19,1000)         
au.start(0) 

ai = IO.PWM(20,1000) 
ai.start(0)

ru = IO.PWM(21,1000)
ru.start(0)

th = IO.PWM(22,1000)
th.start(0)

el = IO.PWM(23,1000) 
el.start(0)
def aux(x):
    au.ChangeDutyCycle(x)

def ail(x):
    ai.ChangeDutyCycle(x)

def rud(x):
    ru.ChangeDutyCycle(x)

def thr(x):
    th.ChangeDutyCycle(x)

def ele(x):
    el.ChangeDutyCycle(x)



def dms(t):
    sleep(t/1000)


def left(speed):


def right(speed):


def up():
    throttle += 1
    thr(throttle)
    dms(10)


def down():
    throttle -= 1
    thr(throttle)
    dms(10)

def takeOff():
    for x in range(0,stThrottle+5):
        thr(x)
        dms(0.1)

    for x in range(stThrottle+5,stThrottle):
        thr(x)
        dms(0.1)

    throttle = stThrottle


def land():
    for x in range(throttle,0):
        thr(x)
        dms(1)

    throttle = 0

def error(median):
    k = np.array(median)/255
    for i in range(4,0):
        if k[i] == 1 :
            return i-4

    for i in range(0,4):
        if k[i+5] == 1 :
            return i

    return 0

def rudderRight():
    ele(minEle)
    rud(rudder)
    delay(1)

def move(error):
    speed = (high/5)*np.absolute(error)

    if error < 0:
        left(speed)

    else :
        right(speed)

    #dms(0.1)


    

def checkShape(median):
    sumL = 0
    sumR = 0
    k = np.array(median[0])/255
    
    for i in range(0,4):
        sumL += k[i]

    for i in range(5,9):
        sumR += k[i]

    if sumL >=3 and sumR >= 3:
        return 1
    elif sumL + sumR >=5 and suml < sumR:

        return 2

    else :
        return 1
        

cap = cv2.VideoCapture(0)
cap.set(3,10)
cap.set(4,10)

while(1):

    # Take each frame
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([30,150,50])
    upper_red = np.array([255,255,180])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    #dilate = cv2.dilate(mask,kernel,iterations = 1)
    median = cv2.medianBlur(mask,15)
    c = checkShape(median)

    if c == 0:
        move(error(median[2])

    if c == 1:
        nElevate = elevate
        for i in range():
             elevate -= 0.1
             ele(elevate)
             dms(1)

        elevate = nElevate
        ele(elevate)
             

    if c == 2:
        rudderRight()

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
    
