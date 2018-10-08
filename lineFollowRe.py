import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
from time import sleep
from time import time
import RPi.GPIO as IO

m = 1000
M = 2000
mid = 1500
stThrottle = 0.0
throttle = 0.0
aileron = 0.0
auxi = 0.0
elevate = 0.0
rudder = 0.0

IO.setwarnings(False)
IO.setmode(IO.BCM)

IO.setup(19,IO.OUT)
IO.setup(20,IO.OUT)
IO.setup(21,IO.OUT)
IO.setup(22,IO.OUT)
IO.setup(23,IO.OUT)

ai = IO.PWM(20,50) 
ai.start(0)

ru = IO.PWM(21,50)
ru.start(0)

th = IO.PWM(22,50)
th.start(0)

el = IO.PWM(23,50) 
el.start(0)


GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
IO.setup(GPIO_TRIGGER, IO.OUT)
IO.setup(GPIO_ECHO, IO.IN)
def clamp(x):
    if x<=1000:
        return 1000

    elif x>=2000:
        return 2000

    else :
        return x

        
def scl(x):
    return x/20000
 
def distance():
    # set Trigger to HIGH
    IO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    sleep(0.00001)
    IO.output(GPIO_TRIGGER, False)
 
    StartTime = time()
    StopTime = time()
 
    # save StartTime
    while IO.input(GPIO_ECHO) == 0:
        StartTime = time()
 
    # save time of arrival
    while IO.input(GPIO_ECHO) == 1:
        StopTime = time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance


def ail(x):
    x = clamp(x)
    d = scl(x)
    ai.ChangeDutyCycle(d)
    return x

def rud(x):
    x = clamp(x)
    d = scl(x)
    ru.ChangeDutyCycle(d)
    return x

def thr(x):
    x = clamp(x)
    d = scl(x)
    th.ChangeDutyCycle(d)
    return x

def ele(x):
    x = clamp(x)
    d = scl(x)
    el.ChangeDutyCycle(d)
    return x



def dms(t):
    sleep(t/1000)


def left():
    ail(mid + 100)
    dms(100)
    ail(mid)

def right():
    ail(mid-100)
    dms(100)
    ail(mid)

def up():
    thr(mid)
    dms(10)


def down():
    throttle -= 1
    thr(throttle)
    dms(10)

def arm():
    thr(m)
    ele(mid)
    ail(mid)
    rud(m)
    sleep(3)

def unarm():
    thr(m)
    ele(mid)
    ail(mid)
    rud(M)
    sleep(3)
    
def takeOff():
    arm()
    for x in range(1000,1650,10):
        thr(x)
        dms(20)
        if distance == height :
            thr(stThrottle)
            dms(20)
            break


def land():
    lnd = stThrottle
    while lnd >=1250:
        lnd = lnd - 5
        thr(lnd)
        dms(20)

    thr(m)
    rud(mid)
    ail(mid)
    ele(mid)
    unarm()
    

def error(median):
    edges = cv2.Canny(median,100,200)
    lines = cv2.HoughLines(edges, 1, np.pi/180,100)
    x = 0
    mini = 0
    #print lines
    
    try:
        mini = abs(lines[0][0][1])
        for line in lines :
            rho, theta = line[0]
            if abs(theta) < mini:
                mini = abs(theta)
                a = np.cos(theta)
                b = np.sin(theta)
                x = (rho - 10*b)/a
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 +1000*(-b))
                y1 = int(x0 +1000*(a))
                x2 = int(x0 -1000*(-b))
                y2 = int(x0 -1000*(a))
            cv2,line(median, (x1,y1), (x2, y2), (0,0,255),2)
            

    except :
        pass
    return [x, mini]

def rudderRight():
    rud(rudder)
    sleep(1)
    rud(mid)
    dms(20)

def move(error, theta):
    q = error*100/68
    aileron = 2000 - q
    ail(aileron)
    r = 3.14/2 - theta*1000/3.14
    rudder = 2000 - r
    rud(rudder)
    dms(60)
    ail(mid)
    rud(mid)
    dms(20)
    

def checkShape(median):
    sumL = 0
    sumR = 0
    k = np.array(median)/255

    if k.sum() > 215040:
        return 5
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
       
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 1
rawCapture = PiRGBArray(camera, size=(640, 480))

sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

#frame = cv2.imread("l1.jpg")
    image = frame.array
    image = cv2.resize(image,(0,0),fx = 0.5,fy = 0.5)
    #frame = cv2.resize(fram, (0,0), fx = 0.015625, fy = 0.020833333)
    ret = 1
    if ret:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([15,50,50])
        upper_yellow = np.array([35,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        median = cv2.medianBlur(mask,21)
        edges = cv2.Canny(median,100,200)
        lines = cv2.HoughLines(edges, 1, np.pi/180,100)
        #median = mask
        #print median
        cv2.imshow('abc', edges)
        cv2.imshow('ab', median)
        #cv2.waitKey(0)
        #print 'bb'
        #c = checkShape(median)
        c = 0
        if c == 0:
            e,t = error(median)
            move(e,t)

        if c == 1:
            n = elevate
            for i in range(0,1):
                elevate -= 0.1
                ele(elevate)
                dms(1)
            elevate = n
            ele(elevate)

        if c == 2:
            rudderRight()
    sleep(0.01)
    k = cv2.waitKey(5) & 0xFF
    rawCapture.truncate(0)
    if k == 27:
        break
cv2.destroyAllWindows()
cap.release()
