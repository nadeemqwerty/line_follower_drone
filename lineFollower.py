import cv2
import numpy as np


def draw_lines(img,lines):
    try:
        for line in lines:
            coords = line[0]
            cv2.line(img, (coords[0], coords[1]), (coords[2], coords[3]), [255,0,0], 3)
    except:
        pass



frame = cv2.imread('l2.jpg')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
lower_red = np.array([15,50,50])
upper_red = np.array([35,255,255])
    
mask = cv2.inRange(hsv, lower_red, upper_red)
kernel = np.ones((2,2),np.uint8)

res = cv2.bitwise_and(frame,frame, mask= median)
edges = cv2.Canny(median, 100, 500)


lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))

    cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)
#draw_lines(frame,lines)
#im2, contours, hierarchy = cv2.findContours(median,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#cv2.drawContours(frame, contours, -1, (0,255,0), 3)
cv2.line(frame, (5,5),(5,500),(255,0,0),2)
line = lines[0][0][1]
print(line)

#erosion = cv2.erode(mask,kernel,iterations = 1)
#dilation = cv2.dilate(mask,kernel,iterations = 1)

#opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)####
#closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

cv2.imshow('Original',frame)
#cv2.imshow('Mask',mask)
#cv2.imshow('canny',edges);
#cv2.imshow('pimage',median)
#cv2.imshow('Dilation',closing)

cv2.waitKey(0)
cv2.destroyAllWindows()

