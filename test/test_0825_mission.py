#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)


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

absolute_altitude = 0


class Controller:
    # initialization method
    def __init__(self):
        pass

    # Callbacks
    def GPS_callback(self, msg):
        global target_gps
        target_gps.latitude = msg.latitude
        target_gps.longitude = msg.longitude

        print('GPS_callback called')


async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
    
    # print("Fetching amsl altitude at home location....")
    # async for terrain_info in drone.telemetry.home():
    #     global absolute_altitude
    #     absolute_altitude = terrain_info.absolute_altitude_m
    #     break

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    flying_alt = absolute_altitude + 2.5 #To fly drone 3m above the ground plane
    #goto_location() takes Absolute MSL altitude 

    mission_items = []
    #parameters = (latitude_deg, longitude_deg, relative_altitude_m, speed_m_s, is_flying_through
    #               gimbal_pitch, gimbal_yaw, camera_action, loiter_time_s, camera_photo_interval_s)
    mission_items.append(MissionItem(47.398039859999997,
                                     8.5455725400000002,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(47.398036222362471,
                                     8.5450146439425509,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(target_gps.latitude,
                                     target_gps.longitude,
                                     1,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return

def main():
    rospy.init_node('SNU_drone', anonymous=True)

    cnt = Controller()

    # rospy.Subscriber('/Target_GPS_msg', Target_GPS, cnt.GPS_callback)
    # rospy.sleep(0.2)

    Target_GPS_Subscriber = rospy.wait_for_message('/Target_GPS_msg', Target_GPS)
    cnt.GPS_callback(Target_GPS_Subscriber)
    rospy.sleep(0.2)


if __name__ == "__main__":
    main()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())