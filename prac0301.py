#!/usr/bin/env python
import sys, rospy, os
from std_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from geographic_msgs import *

bb=True
bb2=True
a = [0,0,0]
b = [0,0,0]
def callback_1(data):
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
    gps_global_get()
    service_name_02 = '/mavros/cmd/takeoff'
    rospy.wait_for_service(service_name_02)
    load_config_srv_prox = rospy.ServiceProxy(service_name_02, CommandTOL)
    req_msg = mavros_msgs.srv.CommandTOLRequest()
    req_msg.min_pitch = 0.0
    req_msg.yaw = 0.0
    req_msg.latitude = b[0]
    req_msg.longitude = b[1]
    req_msg.altitude = 2.0
    resp = load_config_srv_prox(req_msg)
    print("takeoff complete")

def command_move():
    gps_global_get()
    service_name_02 = '/mavros/cmd/takeoff'
    rospy.wait_for_service(service_name_02)
    load_config_srv_prox = rospy.ServiceProxy(service_name_02, CommandTOL)
    req_msg = mavros_msgs.srv.CommandTOLRequest()
    req_msg.min_pitch = 0.0
    req_msg.yaw = 0.0
    req_msg.latitude = 37.453916
    req_msg.longitude = 126.951664
    req_msg.altitude = 2.0
    resp = load_config_srv_prox(req_msg)
    print("takeoff complete")

def command_land():
    gps_global_get()
    service_name_03 = '/mavros/cmd/land'
    rospy.wait_for_service(service_name_03)
    load_config_srv_prox = rospy.ServiceProxy(service_name_03, CommandTOL)
    req_msg = mavros_msgs.srv.CommandTOLRequest()
    req_msg.min_pitch = 0.0
    req_msg.yaw = 0.0
    req_msg.latitude = b[0]
    req_msg.longitude = b[1]
    req_msg.altitude = 0.0
    resp = load_config_srv_prox(req_msg)
    print("land complete")

def callback_gps_local(gps_local_data):
    global a
    a[0] = gps_local_data.pose.pose.position.x
    a[1] = gps_local_data.pose.pose.position.y
    a[2] = gps_local_data.pose.pose.position.z
    print(a)

def gps_local_get():
    topic_name = '/mavros/global_position/local'

    msg = rospy.wait_for_message(topic_name, Odometry)
    callback_2(msg)


def control_position():
    gps_local_get()

    topic_name_3 = '/mavros/setpoint_position/local'
    pub = rospy.Publisher(topic_name_3, PoseStamped, queue_size=10)
    if not rospy.is_shutdown():
        data = PoseStamped()
        data.pose.position.x =  a[0]
        data.pose.position.y =  a[1]
        data.pose.position.z =  a[2]+1
        # data.pose.position.x =  0
        # data.pose.position.y =  0
        # data.pose.position.z =  1
        rate = rospy.Rate(1)
        while pub.get_num_connections() ==0:
            rate.sleep()
        pub.publish(data)
        rate.sleep()
    print(data.pose.position.x)

def callback_gps_global(gps_global_data):
    global b
    b[0] = gps_global_data.latitude
    b[1] = gps_global_data.longitude
    b[2] = gps_global_data.altitude    
    print(b)

def gps_global_get():
    topic_name = '/mavros/global_position/global'
    msg = rospy.wait_for_message(topic_name, NavSatFix)
    callback_gps_global(msg)
 

def control_position_global():
    gps_global_get()
    topic_name = '/mavros/setpoint_raw/global'
    pub = rospy.Publisher(topic_name, GlobalPositionTarget, queue_size=10)
    if not rospy.is_shutdown():
        data = GlobalPositionTarget()
        data.latitude = 37.453916
        data.longitude = 126.951664
        data.altitude = 2
        data.velocity.x = 1
        data.velocity.y = 1
        data.velocity.z = 1
        rate = rospy.Rate(1)

        print(data)
        while pub.get_num_connections() ==0:
            rate.sleep()
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('Drone_PRAC', anonymous=False)
    rospy.sleep(1)

    check_connection()
    if bb==False:
        print("Program terminate")
    command_set_mode("OFFBOARD")

    command_arming(True)
    rospy.sleep(2)

    command_takeoff()
    
    rospy.sleep(2)
    print(b)


    print("change position")

    # control_position_global()
    control_position()
    rospy.sleep(3)

    command_land()

    command_arming(False)

    rospy.sleep(1)
    # control_position(0,0,1)