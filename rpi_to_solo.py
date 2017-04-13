import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

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

# Create a timestamped file to log the data
timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
filename = "flight_" + timestr + ".txt"
f = open(filename, "w+")

# Connect to the Vehicle
vehicle = connect("/dev/ttyS0", baud=921600, wait_ready=True)
vehicle.wait_ready('autopilot_version')

# Function Definitions
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    f.write("\n Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        f.write("\n  Waiting for vehicle to initialise...")
        time.sleep(1)

    f.write("\n Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        f.write("\n  Waiting for arming...")
        time.sleep(1)

    f.write("\n Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        f.write("\n  Altitude: %s " % vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            f.write("\n Reached target altitude")
            break
        time.sleep(1)


# Get data from the PixyCam
def get_pixy_blocks(aLocation1, aLocation2):
    """
    Returns any objects that the Pixy detects in an array [x,y,width,height].
    """
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
    return( [x, y, width, height] )


# Get the distance between two LocationGlobal objects in metres
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(gps_location, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
    the target position. This allows it to be called with different position-setting commands.
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
    The method reports the distance to target every two seconds.
    """

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = gps_location
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    #f.write("\n DEBUG: targetLocation: %s" % targetLocation)
    #f.write("\n DEBUG: targetLocation: %s" % targetDistance)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #f.write("\n DEBUG: mode: %s" % vehicle.mode.name)
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        f.write("\n Distance to target: %s " % remainingDistance)
        if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
            f.write("\n Reached target")
            f.write("Current location:  %s" % vehicle.location.local_frame)
            break;
        time.sleep(2)


# Set altitude to 5 meters above the current altitude
arm_and_takeoff(5)

f.write("\n Set groundspeed to 5m/s.")
vehicle.groundspeed=5

# Fly a path using specific GPS coordinates.
f.write("\n Going to Position 1")
point1 = LocationGlobalRelative(32.66508, -117.03006, 5)
goto(point1)
time.sleep(5)
target = get_pixy_blocks
if target != [0, 0, 0, 0]
    # calculate cm/px
    # calculate error
    # calculate distance
    # calculate angle from North
    # calculate N and E or new GPS point
    # goto(new point)

# drop payload
# toggle GPIO pins
GPIO.output(11, 0)
GPIO.output(13, 1)


f.write("\n Going to Position 2")
point2 = LocationGlobalRelative(32.66542, -117.03020, 5)
goto(point2)
time.sleep(5)
target = get_pixy_blocks
if target != [0, 0, 0, 0]
    # calculate cm/px
    # calculate error
    # calculate distance
    # calculate angle from North
    # calculate N and E or new GPS point
    # goto(new point)

# drop payload
# toggle GPIO pins
GPIO.output(11, 1)
GPIO.output(13, 0)



f.write("\n Going to Position 3")
point3 = LocationGlobalRelative(32.66526, -117.02959, 5)
goto(point3)
time.sleep(5)
target = get_pixy_blocks
if target != [0, 0, 0, 0]
    # calculate cm/px
    # calculate error
    # calculate distance
    # calculate angle from North
    # calculate N and E or new GPS point
    # goto(new point)

# drop payload
# toggle GPIO pins
GPIO.output(11, 1)
GPIO.output(13, 1)

vehicle.mode = VehicleMode("RTL")
f.write("\n Completed")
