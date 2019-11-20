#! /usr/bin/env python
import cv2
import time, os, math
import numpy as np
import rospy
from std_msgs.msg import *
import pyqrcode
from PIL import Image
from pyzbar.pyzbar import decode

cap = cv2.VideoCapture(0)
if cap.get(cv2.CAP_PROP_FRAME_WIDTH) > 2000:
    cap = cv2.VideoCapture(1)

print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

basea = False
baseb = False
basec = False
based = False
basee = False

def sensors_detect():
    global basea
    global baseb
    global basec
    global based
    global basee

    rospy.init_node("read_qrcode", anonymous=False)    

    pub = rospy.Publisher('/hydrone_mission_planner/sensor', String, queue_size=10)
    
    r = rospy.Rate(30) # 10hz

    while not rospy.is_shutdown():

        ret, img = cap.read()

        data = decode(img)
        if len(data) != 0: 
            if data[0].data == "BASE A" and basea == False:
                pub.publish("GREEN")  
                time.sleep(1.0)  
                pub.publish(data[0].data)
                basea = True
                print("BASE A")
            if data[0].data == "BASE B" and baseb == False:
                pub.publish("GREEN")   
                time.sleep(1.0)   
                pub.publish(data[0].data)
                baseb = True
                print("BASE B")
            if data[0].data == "BASE C" and basec == False:
                pub.publish("GREEN")  
                time.sleep(1.0)    
                pub.publish(data[0].data)
                basec = True
                print("BASE C")
            if data[0].data == "BASE D" and based == False:
                pub.publish("GREEN")    
                time.sleep(1.0)  
                pub.publish(data[0].data)
                based = True
                print("BASE D")
            if data[0].data == "BASE E" and basee == False:
                pub.publish("GREEN")  
                time.sleep(1.0)    
                pub.publish(data[0].data)
                basee = True
                print("BASE E")
            

        r.sleep()

if __name__ == "__main__": 
	sensors_detect()
    
