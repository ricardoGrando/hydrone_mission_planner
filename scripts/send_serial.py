#! /usr/bin/env python
#use ttyUSB0 se for o caso, ou se for Windows, COMx.
import serial
import time
import rospy
from std_msgs.msg import *

arduino = serial.Serial("/dev/ttyUSB0",9600)

def send_serial_callback(data):
    if (data.data == "BAD"):
        arduino.write("RED\n")
        print("RED")

    if (data.data == "OK"):
        arduino.write("GREEN\n")
        print("GREEN")

    if (data.data == "GREEN"):
        arduino.write("GREEN\n")
        print("GREEN")

    if (data.data == "BASE A"):
        arduino.write("BASE A\n")
        print("BASE A")

    if (data.data == "BASE B"):
        arduino.write("BASE B\n")
        print("BASE B")

    if (data.data == "BASE C"):
        arduino.write("BASE C\n")
        print("BASE C")

    if (data.data == "BASE D"):
        arduino.write("BASE D\n")
        print("BASE D")
    
    if (data.data == "BASE E"):
        arduino.write("BASE E\n")
        print("BASE E")

if __name__ == "__main__":  
    rospy.init_node("send_serial_node", anonymous=False)   

    rospy.Subscriber("/hydrone_mission_planner/sensor", String, send_serial_callback)

    rospy.spin()


