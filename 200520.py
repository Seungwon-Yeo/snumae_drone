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
a = [0,0]
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
    # print("offmode start")

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

def callback_gps_local(gps_local_data):
    global a
    a[0] = gps_local_data.pose.pose.position.x
    a[1] = gps_local_data.pose.pose.position.y
    print("what is a")
    print(a)

def gps_local_get():
    topic_name_1 = '/mavros/global_position/local'
    rospy.Subscriber(topic_name_1, Odometry, callback_gps_local)
    r = rospy.Rate(20)

def control_position():
    gps_local_get()

    topic_name_3 = '/mavros/setpoint_position/local'
    pub = rospy.Publisher(topic_name_3, PoseStamped, queue_size=10)
    if not rospy.is_shutdown():
        data = PoseStamped()
        data.pose.position.x =  2
        data.pose.position.y =  0
        data.pose.position.z = 0
        rate = rospy.Rate(20)
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
    rospy.Subscriber(topic_name_1, NavSatFix, callback_gps_global)
    r = rospy.Rate(20)
    
def control_position_global():
    gps_global_get()
    topic_name = '/mavros/setpoint_position/global'
    pub = rospy.Publisher(topic_name, GeoPoseStamped, queue_size=10)
    if not rospy.is_shutdown():
        data = GeoPoseStamped()
        data.latitude = b[0]
        data.longitude = b[1]
        data.altitude = b[2]
        rate = rospy.Rate(20)
        while pub.get_num_connections() ==0:
            rate.sleep()
        pub.publish(data)
        rate.sleep()

def cmd_waypoint():
    service_name = '/mavros/mission/push'
    rospy.wait_for_service(service_name)
    wps = []
    load_config_srv_prox = rospy.ServiceProxy(service_name, WaypointPush)
    req_msg = mavros_msgs.msg.Waypoint()
    req_srv = mavros_msgs.srv.WaypointPushRequest()
    req_msg.frame = 0
    req_msg.command = 16
    req_msg.is_current = False
    req_msg.autocontinue = False
    req_msg.param1 =0
    req_msg.param2 = 0
    req_msg.param3 = 0
    req_msg.param4 = 0
    req_msg.x_lat = 37.453916
    req_msg.y_long = 128.951664
    req_msg.z_alt = 2.0
    print(req_msg)
    wps.append(req_msg)

    # req_srv.start_index = 0
    # req_srv.waypoints(req_msg)
    resp = load_config_srv_prox(start_index = 1, waypoints = wps)

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

if __name__ == '__main__':
    print(a)
    rospy.init_node('Drone_PRAC', anonymous=False)
    rospy.sleep(1)

    check_connection()
    if bb==False:
        print("Program terminate")
    command_set_mode("OFFBOARD")

    # check_battery()
    # print("dddd")
    # if bb2==False:
    #     print("Battery Low")

    command_arming(True)
    # command_takeoff()
    rospy.sleep(1)

    # control_position()
    # gps_global_get()

    cmd_waypoint()
    rospy.sleep(2)
    command_set_mode("AUTO.MISSION")
    rospy.sleep(2)

    # control_position_global()

    # rospy.sleep(1)

    # command_land()
    # command_arming(False)

    # rospy.sleep(1)
    # control_position(0,0,1)