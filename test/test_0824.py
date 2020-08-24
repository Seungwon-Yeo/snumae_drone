#!/usr/bin/env python3

# MAVSDK
import asyncio
from mavsdk import System

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
        # self.sp_glob.type_mask = 64

        self.sp_glob.coordinate_frame = 11  # FRAME_GLOBAL_REL_INT
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

        # Record Drone's GPS
        self.drone_GPS_record = []
        # Record Target's GPS
        self.target_GPS_record = []

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
        # rospy.sleep(0.2)
        print(self.global_pos)

    def state(self, msg):
        print('state called')

    def GPS_callback(self, msg):
        self.target_gps.latitude = msg.latitude
        self.target_gps.longitude = msg.longitude
        # self.target_GPS_record.append(self.target_gps)
        print('GPS_callback called')
        # rospy.sleep(0.2)
        # print(self.target_gps)

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y

    def updateSp_glob(self):
        self.sp_glob.latitude = self.target_gps.latitude
        self.sp_glob.longitude = self.target_gps.longitude
        self.sp_glob.latitude = 37.4539751
        self.sp_glob.longitude = 126.9518512

        self.sp_glob.altitude = 3.0
        print('updateSp_glob called')
        print(self.sp_glob)


    def holdSp_glob(self):
        self.sp_glob.latitude = self.global_pos.x
        self.sp_glob.longitude = self.global_pos.y
        self.sp_glob.altitude = 8.0
        print('updateSp_glob called')

    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 1
        self.sp.position.y = self.local_pos.y

    def y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y + 1

# Execution: python3 test_0824.py
async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    cnt = Controller()
    print('before Subscriber')
    # rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)  # cnt.local_pos = Drone's local_position
    rospy.Subscriber('mavros/State', State, cnt.state)
    print('After Subscriber')
    await asyncio.sleep(1)
    # print("Waiting for drone to have a global position estimate...")
    # async for health in drone.telemetry.health():
    #     if health.is_global_position_ok:
    #         print("Global position estimate ok")
    #         break

    # print("Fetching amsl altitude at home location....")
    # async for terrain_info in drone.telemetry.home():
    #     absolute_altitude = terrain_info.absolute_altitude_m
    #     break
    absolute_altitude = 0.0
    print("-- Arming")
    await drone.action.arm()
    print('Absolute Altitude:', absolute_altitude)

    print("-- Taking off")
    # await drone.action.set_takeoff_altitude(3)
    # print (await drone.action.get_takeoff_altitude())
    await drone.action.takeoff()

    await asyncio.sleep(1)
    flying_alt = absolute_altitude + 2.5 #To fly drone 3m above the ground plane
    #goto_location() takes Absolute MSL altitude 
    await drone.action.goto_location(37.4558187, 126.9519536, flying_alt, 0)
    
    # await asyncio.sleep(3)

    # print("-- Landing")
    # await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())