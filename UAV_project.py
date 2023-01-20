import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
import dronekit
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil


#Controls the first servo motor
def servoControl():
    angle = (2+(90/18))
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(38,GPIO.OUT)
    servo1 = GPIO.PWM(38,50) # 38. pin

    servo1.start(0)
    servo1.ChangeDutyCycle(angle)
    time.sleep(2)
    servo1.ChangeDutyCycle(2.5)
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    
    servo1.stop()
    GPIO.cleanup()
    
#Controls the second servo motor
def servo2Control():
    
    GPIO.setmode(GPIO.BOARD)
    angle = (2+(90/18))
    GPIO.setup(40,GPIO.OUT)
    servo2 = GPIO.PWM(40,50) # 40. pin

    servo2.start(0)
    servo2.ChangeDutyCycle(angle)
    time.sleep(2)
    servo2.ChangeDutyCycle(2.5)
    time.sleep(0.5)
    servo2.ChangeDutyCycle(0)

    servo2.stop()
    GPIO.cleanup()

#Access the pixhawk  
def pixhawkDatas(lat,lon,count,f):

    #If pixhawk entire the required location, activate the servo motors respectively
    while True:
        if vehicle.location.global_frame.lat < lat+0.00005 and vehicle.location.global_frame.lat > lat-0.00005 and vehicle.location.global_frame.lon < lon+0.00005 and vehicle.location.global_frame.lon > lon-0.00005:
            f.write("Global Location: %s\n" % vehicle.location.global_frame)        
            if(count==1):
                count += 1
                f.write("Servo 1 is open\n")
                servoControl()
                time.sleep(7)
            else:
                f.write("Servo 2 is open\n")
                servo2Control()
                f.close()
                exit()




vehicle = connect('/dev/serial0', wait_ready=True,baud=921600) #connect vehicle

while True:
    if(vehicle.mode == VehicleMode("AUTO")):#when vehicle is auto, open the camera
        cap = cv.VideoCapture(0)
        break
f = open("logfile.txt", "w") # open a file to write logs
f.write("Script run...\n")
lat = 0
lon = 0
altitude = 0
count = 0

while True:
    ret,frame = cap.read()
    if frame is None:
        break
    original = frame.copy()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    #determines the certain color interval
    lower_red = np.array([160,100,20])
    upper_red = np.array([179,255,255])

    #mask the frame
    red_mask = cv.inRange(hsv,lower_red,upper_red)
    red_color = cv.bitwise_and(frame, frame, mask= red_mask)

    gray = cv.cvtColor(red_color, cv.COLOR_BGR2GRAY)
    gray_blurred = cv.GaussianBlur(gray, (5,5), cv.BORDER_DEFAULT)
    
    #detecting the circles from video
    circles = cv.HoughCircles(gray_blurred, cv.HOUGH_GRADIENT, 1,200,param1=50,
                              param2=20,minRadius=0,maxRadius=100)
   
    #Where camera sees the red circle, saves the location of vehicle
    if circles is not None and count==0:
        circles = np.round(circles[0, :]).astype("int")
        count +=1
        cv.imwrite('Red Circle.png',frame)
        f = open("logfile.txt", "w")
        f.write("Global Location: %s\n" % vehicle.location.global_frame)

        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        altitude = 20
        cap.release()
        time.sleep(10)
        pixhawkDatas(lat,lon,count,f)
    

