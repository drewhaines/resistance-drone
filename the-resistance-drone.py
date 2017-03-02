#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
The Resistance Drone. (Join Us)
Demonstrates how to arm and takeoff in Solo and how to navigate to GPS points.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time


#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect


# Connect to the Vehicle
vehicle = connect("127.0.0.1:14551", wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)


    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

arm_and_takeoff(5)

print "Set default/target airspeed to 3"
vehicle.airspeed = 3


print "Going towards first point for 30 seconds ..."
point1 = LocationGlobalRelative(32.773902, -117.072860, 5)
vehicle.simple_goto(point1)
# sleep so we can see the change in map
time.sleep(30)

print "Going towards second point for 30 seconds ..."
point2 = LocationGlobalRelative(32.773523, -117.072120, 5)
vehicle.simple_goto(point2)
# sleep so we can see the change in map
time.sleep(30)

print "Going towards thrid point for 30 seconds ..."
point3 = LocationGlobalRelative(32.773180, -117.072764, 5)
vehicle.simple_goto(point3)
# sleep so we can see the change in map
time.sleep(30)

print "Going back home."
home = LocationGlobalRelative(32.773632, -117.073654, 5)
vehicle.simple_goto(home)
# sleep so we can see the change in map
time.sleep(30)

print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
