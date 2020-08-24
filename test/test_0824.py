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


# Instantiate a Target_GPS message
target_gps = Target_GPS()

# A Message for the current local position of the drone
local_pos = Point(0.0, 0.0, 0.0)
global_pos = Point(0.0, 0.0, 0.0)

class Controller:
    # initialization method
    def __init__(self):
        pass

    # Callbacks
    def state(self, msg):
        target_gps.latitude = 2
        target_gps.longitude = 3
        print('state called')

    def GPS_callback(self, msg):
        global target_gps
        target_gps.latitude = msg.latitude
        target_gps.longitude = msg.longitude

        print('GPS_callback called')

# Execution: python3 test_0824.py
async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

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

    # await drone.action.goto_location(37.453989, 126.9517722, flying_alt, 0)
    # await asyncio.sleep(1)
    
    await drone.action.goto_location(target_gps.latitude, target_gps.longitude, flying_alt, 0)
    await asyncio.sleep(1)

    # print("-- Landing")
    # await drone.action.land()

def main():
    rospy.init_node('SNU_drone', anonymous=True)

    cnt = Controller()
    # rospy.Subscriber('/mavros/state', State, cnt.state)
    # rospy.sleep(0.2)
    
    rospy.Subscriber('/Target_GPS_msg', Target_GPS, cnt.GPS_callback)
    rospy.sleep(0.2)

if __name__ == "__main__":
    main()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())