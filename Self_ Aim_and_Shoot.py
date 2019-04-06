import cv2
import math
import serial
import numpy as np
import struct
cap=cv2.VideoCapture(0)
ser = serial.Serial()
ser.baudrate = 250000#the baud rate over which the arduino and python will communicate
ser.port = 'COM11' # change it for your owm com port
ser.open()
pre_servo_lower=0
pre_servo_upper=0
pre_servo_lower1=0
pre_servo_upper1=0
pre_center_x=0
pre_center_y=0
start_time=0
kernel = np.ones((7,7),np.uint8)
global center_x,center_y
def nothing(x):
	pass
def calibrateColor(color, def_range):   # this function sets the color of the object that you want to detect
    global kernel
    name = 'Calibrate ' + color
    cv2.namedWindow(name)
    cv2.createTrackbar('Hue', name, def_range[0][0] + 20, 180,nothing)
    cv2.createTrackbar('Sat', name, def_range[0][1], 255,nothing)
    cv2.createTrackbar('Val', name, def_range[0][2], 255,nothing)
    while (1):
        ret, frameinv = cap.read(0)
        frame = cv2.flip(frameinv, 1)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hue = cv2.getTrackbarPos('Hue', name)
        sat = cv2.getTrackbarPos('Sat', name)
        val = cv2.getTrackbarPos('Val', name)

        lower = np.array([hue - 20, sat, val])
        upper = np.array([hue + 20, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)
        eroded = cv2.erode(mask, kernel, iterations=1)
        dilated = cv2.dilate(eroded, kernel, iterations=1)

        cv2.imshow(name, dilated)

        k = cv2.waitKey(5) & 0xFF
        if k == ord(' '):
            cv2.destroyWindow(name)
            return np.array([[hue - 20, sat, val], [hue + 20, 255, 255]])
        elif k == ord('d'):
            cv2.destroyWindow(name)
            return def_range
#blue_range = np.array([79, 115, 150],[120, 255, 255])
red_range = np.array([[158,85,72],[180,255,255]])
#blue_range = calibrateColor('Red', blue_range)
red_range = calibrateColor('red', red_range)
while(1):
    _,img=cap.read()
    img=cv2.flip(img,1)#to get a flipped image
    height,width,depth=img.shape
    img=cv2.resize(img,(height*3,int(width*1.1)))
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    kernal = np.ones((5, 5), "uint8")
    mask = cv2.inRange(hsv, red_range[0], red_range[1])
    #mask1=cv2.(hsv,blue_range)
    eroded = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(eroded, kernel, iterations=1)

    (contours,hierarchy)=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for pic,contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if(area>100):
            x,y,w,h=cv2.boundingRect(contour)
            img=cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(img,"",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255))
            center_x=x+(w/2)        #calculate the center X coordinate of the detected object
            center_y=y+(h/2)        #calculate the center Y coordinate of the detected object
           # print(center_x)
            servo_x=math.ceil(math.degrees(math.atan(center_x/600)))# convert the pixel coordinate into angles
            if servo_x>pre_servo_lower and servo_x<pre_servo_upper :

                pass
            else :
                #print(servo_x)
                ser.write(str(servo_x).encode()+'\n'.encode())   #write the angle values on the X axis servo 
                ser.write('a'.encode()+'\n'.encode())           #write 'a' to distinguish the anfle values for the X axis only
                pre_servo_lower=servo_x-3                       #takes a range of -3 to +3 pixels to reduce the number of values written
                pre_servo_upper=servo_x+3
            servo_y = math.ceil(math.degrees(math.atan(center_y/400))) # convert the pixel coordinate into angles
            if servo_y>pre_servo_lower1 and servo_y<pre_servo_upper1 :
                if (center_x - pre_center_x) ** 2 <= 50 and (center_y - pre_center_y) ** 2 <= 50:
                    start_time = start_time+1   #it counts the time for which the object remained stationary 
                #print('no values written for y')
            else :
                ser.write(str(servo_y).encode()+'\n'.encode())   #write the angle values on the Y axis servo
                ser.write('b'.encode()+'\n'.encode())           # write 'b' to distinguish the angle value for Y axis only
                pre_servo_lower1=servo_y-3
                pre_servo_upper1=servo_y+3
                start_time=0;
            pre_center_x=center_x
            pre_center_y=center_y
            if start_time>=15:
                ser.write('c'.encode() + '\n'.encode())
                print("shoot")
                start_time=0
    cv2.imshow('Color Tracking',img)
    k=cv2.waitKey(50)& 0xFF
    if k==27:   #press esc key to close the window
        break
cv2.destroyAllWindows()