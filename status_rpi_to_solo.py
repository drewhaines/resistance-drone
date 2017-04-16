#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py:

Demonstrates how to get and set vehicle state and parameter information,
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
from math import *

# Pixy Stuff
from pixy import *
from ctypes import *

# Initialize Pixy Interpreter thread #
pixy_init()

class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

blocks = BlockArray(30)
frame  = 0

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

GPIO.output(17, GPIO.LOW)
GPIO.output(27, GPIO.LOW)

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

vehicle = connect("/dev/ttyS0", baud=921600, wait_ready=True)

timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
filename = "flight_" + timestr + ".txt"
f = open(filename, "w+")

# Get all vehicle attributes (state)
f.write("\nGet all vehicle attribute values:")
f.write("\n Autopilot Firmware version: %s" % vehicle.version)
f.write("\n   Major version number: %s" % vehicle.version.major)
f.write("\n   Minor version number: %s" % vehicle.version.minor)
f.write("\n   Patch version number: %s" % vehicle.version.patch)
f.write("\n   Release type: %s" % vehicle.version.release_type() )
f.write("\n   Release version: %s" % vehicle.version.release_version() )
f.write("\n   Stable release?: %s" % vehicle.version.is_stable() )
f.write("\n Autopilot capabilities")
f.write("\n   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
f.write("\n   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
f.write("\n   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
f.write("\n   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
f.write("\n   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
f.write("\n   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
f.write("\n   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
f.write("\n   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
f.write("\n   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
f.write("\n   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
f.write("\n   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
f.write("\n   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
f.write("\n   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
f.write("\n   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
f.write("\n Global Location: %s" % vehicle.location.global_frame)
f.write("\n Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
f.write("\n Local Location: %s" % vehicle.location.local_frame)
f.write("\n Attitude: %s" % vehicle.attitude)
f.write("\n Velocity: %s" % vehicle.velocity)
f.write("\n GPS: %s" % vehicle.gps_0)
f.write("\n Gimbal status: %s" % vehicle.gimbal)
f.write("\n Battery: %s" % vehicle.battery)
f.write("\n EKF OK?: %s" % vehicle.ekf_ok )
f.write("\n Last Heartbeat: %s" % vehicle.last_heartbeat)
f.write("\n Rangefinder: %s" % vehicle.rangefinder)
f.write("\n Rangefinder distance: %s" % vehicle.rangefinder.distance)
f.write("\n Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
f.write("\n Heading: %s" % vehicle.heading)
f.write("\n Is Armable?: %s" % vehicle.is_armable)
f.write("\n System status: %s" % vehicle.system_status.state)
f.write("\n Groundspeed: %s" % vehicle.groundspeed )   # settable
f.write("\n Airspeed: %s" % vehicle.airspeed )   # settable
f.write("\n Mode: %s" % vehicle.mode.name  )  # settable
f.write("\n Armed: %s" % vehicle.armed )   # settable

# Pixy test
count = pixy_get_blocks(30, blocks)
sums = [0, 0, 0, 0]       # x, y, width, height
average = [0, 0, 0, 0]   # x, y, width, height

if count > 0:
    # Blocks found!
    f.write("\n\n Blocks found!")
    f.write("\n frame %3d:" % frame)
    frame = frame + 1

    for index in range (0, count):
        sums[0] += blocks[index].x
        sums[1] += blocks[index].y
        sums[2] += blocks[index].width
        sums[3] += blocks[index].height

    average[0] = (sums[0]/count)
    average[1] = (sums[1]/count)
    average[2] = (sums[2]/count)
    average[3] = (sums[3]/count)

x = average[0]
y = average[1]
width = average[2]
height = average[3]
f.write( '\n [X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (x, y, width, height))

# drop payload by toggling GPIO pins
f.write("\n Drop GPIO 1!")
GPIO.output(17, GPIO.LOW)
GPIO.output(27, GPIO.HIGH)
time.sleep(3)

# drop payload by toggling GPIO pins
f.write("\n Drop GPIO 2!")
GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.LOW)
time.sleep(3)

# drop payload by toggling GPIO pins
f.write("\n Drop GPIO 3!")
GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.HIGH)
time.sleep(3)




f.write("Completed")
GPIO.cleanup()
