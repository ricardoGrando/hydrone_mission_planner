#! /usr/bin/env python
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *

local_position_pose = 0

def local_position_callback(data):
    local_position_pose = data
    # print(local_position_pose)

def mode_callback(data):
    if data.mode == "ACRO": 
        call_set_mode("GUIDED", 4)   
        mission_planner()        

rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)
set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
activate_sensor = rospy.Publisher("/hydrone_mission_planner/sensor/activate", Bool, queue_size=10)

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.wait_for_service("/mavros/set_mode")

        print(service(mode_ID, mode))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e        

def call_arming(value):
    try:
        service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        rospy.wait_for_service("/mavros/cmd/arming")

        print(service(value))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def call_takeoff(altitude):
    try:
        service = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        rospy.wait_for_service("/mavros/cmd/takeoff")

        print(service(0.0, 0.0, 0.0, 0.0, altitude))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def set_target_position(x,y,z):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # pose.pose.orientation.x = local_position_pose.pose.orientation.x
    # pose.pose.orientation.y = local_position_pose.pose.orientation.y
    # pose.pose.orientation.z = local_position_pose.pose.orientation.z
    # pose.pose.orientation.w = local_position_pose.pose.orientation.w

    set_point_pub.publish(pose)

def call_land(altitude):
    try:
        service = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        rospy.wait_for_service("/mavros/cmd/land")

        print(service(0.0, 0.0, 0.0, 0.0, altitude))
        
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

def pub_reset_gps():
    msg = GeoPointStamped()
    reset_gps.publish(msg)

def goto_to_aerial_base1():

    call_arming(True)
    print("Armed")

    call_takeoff(2.0)
    print("Took off")

    time.sleep(10)

    set_target_position(2.8, 0.0, 2.0)
    print("Target set")

    time.sleep(20)
    
    call_land(2.0)
    print("Land")

    time.sleep(10)

def goto_to_aerial_base2():

    call_set_mode("GUIDED", 4)
    print("Mode set")

    call_arming(True)
    print("Armed")

    call_takeoff(1.0)
    print("Took off")

    time.sleep(10)

    # pegar medidas certas
    set_target_position(4.0, -7.0, 1.0)
    print("Target set")

    time.sleep(30)
    
    call_land(1.0)
    print("Land")

    time.sleep(10)

def return_coastal_base():

    call_set_mode("GUIDED", 4)
    print("Mode set")

    call_arming(True)
    print("Armed")

    call_takeoff(1.0)
    print("Took off")

    time.sleep(10)

    set_target_position(0.0, 0.0, 1.0)
    print("Target set")

    time.sleep(10)
    
    call_land(0.80)
    print("Land")

    time.sleep(10)

def goto_corner_can1():

    call_arming(True)
    print("Armed")

    call_takeoff(1.0)
    print("Took off")

    time.sleep(10)

    set_target_position(0.0, -6.6, 1.0)
    print("Target set")

    time.sleep(30)

def goto_corner_can2():

    activate_sensor.publish(True)

    set_target_position(6.0, 0.0, 1.0)
    print("Target set")

    time.sleep(20)

def goto_center():
    
    activate_sensor.publish(False)

    set_target_position(3.0, -3.0, 1.0)
    print("Target set")

    time.sleep(10)    

def return_home():

    set_target_position(0.0, 0.0, 1.5)
    print("Target set")

    time.sleep(10)

    call_land(0.80)
    print("Land")

    time.sleep(10)

def goto_to_aerial_base1_qrcode():

    call_arming(True)
    print("Armed")

    call_takeoff(2.0)
    print("Took off")

    time.sleep(10)

    set_target_position(2.8, 0.0, 2.0)
    print("Target set")

    time.sleep(20)

def goto_land_base_1():
    
    set_target_position(2.3, -2.0, 2.0)
    print("Target set")

    time.sleep(10) 

    set_target_position(2.3, -2.0, 0.5)
    print("Target set")

    time.sleep(20) 

    set_target_position(2.3, -2.0, 1.5)
    print("Target set")

    time.sleep(10) 

def goto_land_base_2():

    set_target_position(5.85, -3.85, 1.5)
    print("Target set")

    time.sleep(10) 
    
    set_target_position(5.85, -3.85, 0.75)
    print("Target set")

    time.sleep(20) 

    set_target_position(5.85, -3.85, 1.5)
    print("Target set")

    time.sleep(10) 

def goto_land_base_3():

    set_target_position(2.95, -5.6, 1.5)
    print("Target set")

    time.sleep(10) 
    
    set_target_position(2.95, -5.6, 0.5)
    print("Target set")

    time.sleep(20) 

    set_target_position(2.95, -5.6, 1.5)
    print("Target set")

    time.sleep(10) 
    
def mission_planner():
    call_set_mode("GUIDED", 4)
    print("Mode set")

    time.sleep(1)

    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    pub_reset_gps()

    time.sleep(5)

    call_arming(True)
    print("Armed")

    time.sleep(5)


    # Tarefa 1

    # goto_to_aerial_base1()

    # return_coastal_base()

    # goto_to_aerial_base1()

    # # goto_to_aerial_base2()

    # return_coastal_base()

    # goto_to_aerial_base1()

    # return_coastal_base()


    # # Tarefa2

    # goto_corner_can1()

    # goto_corner_can2()

    # goto_center()goto_to_aerial_base1_qrcode()


    # Tarefa 3

    goto_to_aerial_base1_qrcode()

    goto_land_base_1()

    goto_land_base_2()

    goto_land_base_3()

    return_home()

    
    # Tarefa 4

    #goto_to_aerial_base1_qrcode()

    #goto_land_base_1()

    #goto_land_base_2()

    #goto_land_base_3()

    #return_home()


if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    

    rospy.Subscriber("/mavros/state", State, mode_callback)

    rospy.spin()
