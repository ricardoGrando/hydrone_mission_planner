#! /usr/bin/env python
import cv2
import time, os, math
import numpy as np
import rospy
from std_msgs.msg import *
from PIL import Image
import pytesseract
import argparse
import os
import digits_class as dg

cap = cv2.VideoCapture(0)
if cap.get(cv2.CAP_PROP_FRAME_WIDTH) > 2000:
    cap = cv2.VideoCapture(1)

if __name__ == "__main__": 
    rospy.init_node("read_qrcode", anonymous=False)    

    pub = rospy.Publisher('/hydrone_mission_planner/sensor', String, queue_size=10)

    r = rospy.Rate(30) # 10hz
    
    while not rospy.is_shutdown():

        ret, img = cap.read()        

        t# image para a classe
	mark.setImage(img, 0.)
	
	# get reference frame
	success, val1, val2 = mark.getRefFrame()
	
	if success:
             	pub_detect.publish("GREEN")
             	print(GREEN)
       
        
        r.sleep()
