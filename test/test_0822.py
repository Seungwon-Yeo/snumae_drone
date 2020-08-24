#!/usr/bin/env python3

import asyncio
from mavsdk import System

# Execution: python3 test_0822.py
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
    await drone.action.goto_location(37.4558187, 126.9519536, flying_alt, 0)
    
    # await asyncio.sleep(3)

    # print("-- Landing")
    # await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())