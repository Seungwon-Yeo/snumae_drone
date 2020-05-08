#!/usr/bin/env python
import sys, rospy, os
from std_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
bb=True
bb2=True


def callback_1(data):
    global bb
    if (data.connected == False):
        print("not connected")
        bb = False


def check_connection():
    topic_name_1 = '/mavros/state'
    # rospy.Subscriber(topic_name_1, State, callback_1)
    msg01 = rospy.wait_for_message(topic_name_1, State)
    callback_1(msg01)
    print("yes")
    
def callback_2(data_2):
    global bb2
    if (data_2.voltage <= 14.6):
        print("low battery")
        bb2 = False

def check_battery():
    topic_name_2 = '/mavros/battery'
    # rospy.Subscriber(topic_name_2, BatteryState, callback_2)
    msg02 = rospy.wait_for_message(topic_name_2, BatteryState)
    callback_2(msg02)

def commander():
    topic_name_2 = '/mavros/'

def command_set_mode(config_0):
    service_name = '/mavros/set_mode'
    rospy.wait_for_service(service_name)
    load_config_srv_prox = rospy.ServiceProxy(service_name, SetMode)
    req_msg = mavros_msgs.srv.SetModeRequest()
    req_msg.base_mode = 0
    req_msg.custom_mode = "config_0"
    resp = load_config_srv_prox(req_msg)
    print("offmode start")

def command_arming(config_1):
    service_name = '/mavros/cmd/arming'
    rospy.wait_for_service(service_name)
    load_config_srv_prox = rospy.ServiceProxy(service_name, CommandBool)
    req_msg = mavros_msgs.srv.CommandBoolRequest()
    req_msg.value = config_1
    resp = load_config_srv_prox(req_msg)

def command_takeoff():
    service_name_02 = '/mavros/cmd/takeoff'
    rospy.wait_for_service(service_name_02)
    load_config_srv_prox = rospy.ServiceProxy(service_name_02, CommandTOL)
    req_msg = mavros_msgs.srv.CommandTOLRequest()
    req_msg.min_pitch = 0.0
    req_msg.yaw = 0.0
    req_msg.latitude = 0.0
    req_msg.longitude = 0.0
    req_msg.altitude = 2.0
    resp = load_config_srv_prox(req_msg)
    print("takeoff complete")

def command_land():
    service_name_03 = '/mavros/cmd/land'
    rospy.wait_for_service(service_name_03)
    load_config_srv_prox = rospy.ServiceProxy(service_name_03, CommandTOL)
    req_msg = mavros_msgs.srv.CommandTOLRequest()
    req_msg.min_pitch = 0.0
    req_msg.yaw = 0.0
    req_msg.latitude = 0.0
    req_msg.longitude = 0.0
    req_msg.altitude = 0.0
    resp = load_config_srv_prox(req_msg)
    print("land complete")

def control_position(x,y,z):
    topic_name_3 = '/mavros/setpoint_position/local'
    pub = rospy.Publisher(topic_name_3, PoseStamped, queue_size=10)
    if not rospy.is_shutdown():
        data = PoseStamped()
        data.pose.position.x = x
        data.pose.position.y = y
        data.pose.position.z = z
        rate = rospy.Rate(1)
        while pub.get_num_connections() ==0:
            rate.sleep()
        pub.publish(data)
        rate.sleep()
    print("move to x y z")



if __name__ == '__main__':
    print("can start?")
    rospy.init_node('Drone_PRAC', anonymous=False)
    rospy.sleep(1)

    check_connection()
    if bb==False:
        print("Program terminate")
    command_set_mode("OFFBOARD")

    check_battery()
    print("dddd")
    if bb2==False:
        print("Battery Low")
    command_arming(True)
    command_takeoff()
    rospy.sleep(1)

    control_position(0,0,1)

    rospy.sleep(1)

    command_land()
    command_arming(False)

    rospy.sleep(3)
    # control_position(0,0,1)