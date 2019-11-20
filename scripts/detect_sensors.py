#! /usr/bin/env python
import sensors_class as sc
import cv2
import time, os, math
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import *

cap = cv2.VideoCapture(0)
if cap.get(cv2.CAP_PROP_FRAME_WIDTH) > 2000:
    cap = cv2.VideoCapture(1)

mark = sc.Sensors()

flag = False

def activate_sensor_detection(data):
    global flag
    if data.data == True:
        flag = True
    else:
        flag = False


def sensors_detect():
    global flag

    rospy.init_node("sensors_detect", anonymous=False)

    rospy.Subscriber("/hydrone_mission_planner/sensor/activate", Bool, activate_sensor_detection)

    pub = rospy.Publisher('/hydrone_mission_planner/sensor', String, queue_size=10)

    r = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():

        ret, img = cap.read()

        # image para a classe
        mark.setImage(img, 0.)

        # get reference frames
        R1, T1, success1, R2, T2, success2 = mark.getRefFrameSMarks()

        print(T1)

        print(flag, success1)

        if success1 == True:
		if flag == True:                	
			pub.publish("BAD")
        	print("BAD")

    	if success2 == True:
		if flag == True:        
        		pub.publish("OK")
        	print("OK")

        #cv2.imshow('image', mark.img)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

        #mark.show()
        #print("############################")

        r.sleep()

if __name__ == "__main__": 

    sensors_detect()      
