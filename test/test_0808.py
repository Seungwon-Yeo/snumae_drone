#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped  # /opt/ros/melodic/share/geometry_msgs
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix

# import a message from Raspberry pi CAMERA
# Message name: /target_msg
from targetDetection_topic.msg import TargetPosition
# import a message from Raspberry pi GPS
# Message name: /Target_GPS_msg
from GPS_topic.msg import Target_GPS


# Flight modes class
# Flight modes are activated using ROS services

class Controller:
    # initialization method
    def __init__(self):
        # Instantiate a "mavros/setpoint_raw/local" message
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)  # set the flag to use position setpoints and yaw angle
        self.sp.coordinate_frame = 1  # FRAME_LOCAL_NED

        # Instantiate a "mavros/setpoint_raw/global" message
        self.sp_glob = GlobalPositionTarget()
        self.sp_glob.type_mask = int('111111111000', 2)
        self.sp_glob.coordinate_frame = 6  # FRAME_GLOBAL_INT
        self.sp_glob.latitude = 0
        self.sp_glob.longitude = 0
        self.sp_glob.altitude = 0

        # Instantiate a Target_GPS message
        self.target_gps = Target_GPS()

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.global_pos = Point(0.0, 0.0, 0.0)

        # initial values for setpoints
        self.sp.position.x = 0.0  # geometry_msgs/Point position
        self.sp.position.y = 0.0

    # Callbacks
    ## local position callback
    def posCb(self, msg):  # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
        self.local_pos.x = msg.pose.position.x  # PoseStamped(msg).Pose(pose).Point(position).x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        print('posCb called')

    def posCb_glob(self, msg_glob):  # rospy.Subscriber('mavros/global_position/global', NavSatFix, cnt.posCb_glob)
        self.global_pos.x = msg_glob.latitude  # NavSatFix(msg_glob).latitude -> global_pos.x = latitude
        self.global_pos.y = msg_glob.longitude
        self.global_pos.z = msg_glob.altitude
        print('posCb_glob called')

    def GPS_callback(self, msg):
        self.target_gps.latitude = msg.latitude
        self.target_gps.longitude = msg.longitude
        print('GPS_callback called')
        print(self.target_gps)

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y

    def updateSp_glob(self):
        self.sp_glob.latitude = self.target_gps.latitude
        self.sp_glob.longitude = self.target_gps.longitude
        self.sp_glob.altitude = 0
        print('updateSp_glob called')

    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 1
        self.sp.position.y = self.local_pos.y

    def y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y + 1


class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
            print('Arming Succeeded')
        except rospy.ServiceException, e:
            print "Service arming call failed: %s" % e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print('Changed to OffboardMode')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set." % e

    def setTakeoff(self, global_pos):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(latitude=global_pos.x, longitude=global_pos.y, altitude=1)
            print('TakeOff Succeeded')
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s" % e

    def setAutoMissionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.MISSION')
            print('Changed to Auto Mission Mode')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. AutoMission Mode could not be set." % e

    def wpPush(self, wps):
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
            wpPushService(start_index = 0, waypoints = wps) #start_index = the index at which we want the mission to start
            print "Waypoint Pushed"
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s" % e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
            print('Auto Landing..')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set." % e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
            print('Disarming Succeeded')
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s" % e

class wpMissionCnt:
    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self, frame, command, is_current, autocontinue, param1, param2, param3, param4, x_lat, y_long, z_alt):
        self.wp.frame = frame # FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        self.wp.command = command  # VTOL TAKEOFF = 84, NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
        self.wp.is_current = is_current
        self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        self.wp.param1 = param1 # Hold
        self.wp.param2 = param2 # Accept Radius
        self.wp.param3 = param3 # Pass Radius
        self.wp.param4 = param4 # Yaw
        self.wp.x_lat = x_lat # Latitude
        self.wp.y_long =y_long
        self.wp.z_alt = z_alt #relative altitude.

        return self.wp

# Main function
def main():
    # initiate node
    rospy.init_node('SNU_drone', anonymous=True)

    # controller object
    cnt = Controller()

    # flight mode object
    modes = fcuModes()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Waypoint object 
    wayp0 = wpMissionCnt()
    wayp1 = wpMissionCnt()
    wps = [] #List to store waypoints

    # Store Target's GPS & Drone's GPS
    T_gps = []
    D_gps = []

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)  # cnt.local_pos = Drone's local_position
    rospy.sleep(0.2)
    rospy.Subscriber('mavros/global_position/global', NavSatFix, cnt.posCb_glob)  # cnt.global_pos = Drone's GPS position(lat, long, alt)
    rospy.sleep(0.2)
    rospy.Subscriber('/Target_GPS_msg', Target_GPS, cnt.GPS_callback)
    rospy.sleep(0.2)
    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    rospy.sleep(0.2)
    sp_glob_pub = rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
    rospy.sleep(0.2)

    # Set Waypoint
    w = wayp0.setWaypoints(3,84,True,True,0.0,0.0,0.0,float('nan'), cnt.global_pos.x, cnt.global_pos.y , 1)
    wps.append(w)

    w = wayp1.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'), cnt.target_gps.latitude, cnt.target_gps.longitude, 1)
    wps.append(w)

    modes.wpPush(wps)
    print(wps)

    ##########################################
    #### Now the Drone starts operation!! ####
    ##########################################

    # # Arm the drone
    modes.setArm()
    rospy.sleep(1)

    # # Save current GPS position
    # T_gps.append(cnt.target_gps.latitude)
    # T_gps.append(cnt.target_gps.longitude)
    # D_gps.append(cnt.global_pos.latitude)
    # D_gps.append(cnt.global_pos.longitude)
    # D_gps.append(cnt.global_pos.altitude)

    # # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k = 0
    while k < 10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # # TakeOff
    modes.setTakeoff(cnt.global_pos)
    rospy.sleep(3)

    # ## activate AUTO.MISSION mode
    # modes.setAutoMissionMode()
    # rospy.sleep(3)

    # # Update sp_glob to Target's GPS
    # cnt.updateSp_glob()
    # print(cnt.sp_glob)
    # rospy.sleep(1)

    # # # # Update sp to Target's local position
    cnt.updateSp()
    rospy.sleep(1)
    cnt.x_dir()
    rospy.sleep(0.2)

    # # # Move the drone to Target's GPS position
    # sp_glob_pub.publish(cnt.sp_glob)
    # rospy.sleep(5)

    # # Move the drone to Target's local position by Camera info
    sp_pub.publish(cnt.sp)
    rospy.sleep(3)

    # Land the drone
    modes.setAutoLandMode()
    rospy.sleep(3)

    # Disarm the drone
    modes.setDisarm()
    rospy.sleep(2)

    # with open("Target_GPS.txt", "w") as f:
    #     f.write('Target_GPS \n'.join(T_gps))
    #     f.write('Drone_GPS\n'.join(D_gps))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass