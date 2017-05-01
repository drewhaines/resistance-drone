import RPi.GPIO as GPIO
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
from math import *

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

GPIO.setup(16, GPIO.IN)
GPIO.setup(20, GPIO.IN
GPIO.setup(21, GPIO.IN)


# Pixy Stuff
from pixy import *
from ctypes import *

# Initialize Pixy
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
def get_pixy_blocks():
    """
    Returns any objects that the Pixy detects in an array [x,y,width,height].
    """
    count = pixy_get_blocks(30, blocks)
    sums = [0, 0, 0, 0]       # x, y, width, height
    average = [0, 0, 0, 0]   # x, y, width, height

    if count > 0:
        # Blocks found!
        f.write("\n\n Blocks found!")

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




def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;





def pixy_goto():
    """
    Moves the vehicle using data from the pixy.
    """
    pixy_count = 0

    while pixy_count < 5:
        target = get_pixy_blocks() # returns [x, y, width, height]

        if target != [0, 0, 0, 0]:
            pixy_count = 5
        elif target == [0, 0, 0, 0] and pixy_count == 0:
            # say the target on the left the drone
            target = [50, 101, 10, 10]
            pixy_count += 1
        elif target == [0, 0, 0, 0] and pixy_count == 1:
            # say the target is behind drone's original location
            target = [0, 199, 10, 10]
            pixy_count += 1
        elif target == [0, 0, 0, 0] and pixy_count == 2:
            # say the target is to the right of the drone's original location
            target = [0, 101, 10, 10]
            pixy_count += 1
        elif target == [0, 0, 0, 0] and pixy_count == 3:
            # say the target infront of the drone's original location
            target = [0, 101, 10, 10]
            pixy_count += 1
        elif target == [0, 0, 0, 0] and pixy_count == 4:
            # go back to the original location to drop the payload
            target = [0, 199, 10, 10]
            pixy_count += 1


        # calculate the average cm/px with an object of 30cm X 30cm
        cm_per_pixel_1 = 20/target[2]
        cm_per_pixel_2 = 20/target[3]
        cm_per_pixel = (cm_per_pixel_1+cm_per_pixel_2)/2
        f.write("\n  cm_per_pixel: %3d " % cm_per_pixel)

        # calculate error
        error_x = float(target[0]-160)
        error_y = float((target[1]-100)*(-1)) # multiple by -1 because Pixy uses (0,0) as top left
        hypotenuse = sqrt(error_x**2 + error_y**2)
        f.write("\n  error_x: %3d " % error_x)
        f.write("\n  error_y: %3d " % error_y)
        f.write("\n  hypotenuse: %.3f " % hypotenuse)

        # calculate angle from Pixy
        if (error_x >= 0 and error_y >= 0):  # quadrant 1
            f.write("\n pixy quadrant 1")
            pixy_angle = atan(float(error_x)/ float(error_y)) * 57.2957795
        elif (error_x >= 0 and error_y < 0):   # quadrant 2
            f.write("\n pixy quadrant 2")
            pixy_angle = (atan(float(-error_y)/ float(error_x)) * 57.2957795) + 90
        elif (error_x < 0 and error_y < 0):  # quadrant 3
            f.write("\n pixy quadrant 3")
            pixy_angle = (atan(float(-error_x)/ float(-error_y)) * 57.2957795) + 180
        elif (error_x < 0 and error_y >= 0):   # quadrant 4
            f.write("\n pixy quadrant 4")
            pixy_angle = (atan(float(error_y)/ float(-error_x)) * 57.2957795) +270

        if pixy_angle < 0:
            pixy_angle += 360.00
        if pixy_angle > 360:
            pixy_angle -= 360.00
        f.write("\n  pixy_angle: %3d " % pixy_angle)

        # calculate angle from North
        angle_from_north = vehicle.heading + pixy_angle
        if angle_from_north > 360:
            angle_from_north -= 360.00
        f.write("\n  angle_from_north: %3d " % angle_from_north)


        # calculate the pixles N and E
        if angle_from_north <= 90:   # quadrant 1
            f.write("\n pixy quadrant 1")
            pixels_north = hypotenuse*cos(radians(angle_from_north))
            pixels_east = hypotenuse*sin(radians(angle_from_north))
        elif angle_from_north <= 180:   # quadrant 2
            angle = angle_from_north-90
            pixels_north = -(hypotenuse*sin(radians(angle)))
            pixels_east = hypotenuse*cos(radians(angle))
        elif angle_from_north <= 270:   # quadrant 3
            angle = angle_from_north-180
            pixels_north = -(hypotenuse*cos(radians(angle)))
            pixels_east = -(hypotenuse*sin(radians(angle)))
        elif angle_from_north < 360:   # quadrant 4
            angle = angle_from_north-270
            pixels_north = hypotenuse*sin(radians(angle))
            pixels_east = -(hypotenuse*cos(radians(angle)))
        f.write("\n  pixels_north: %.5f " % pixels_north)
        f.write("\n  pixels_east: %.5f " % pixels_east)


        # convert to distance in meters N and E
        distance_north = (pixels_north*cm_per_pixel)/100
        distance_east = (pixels_east*cm_per_pixel)/100
        f.write("\n  distance_north: %.5f " % distance_north)
        f.write("\n  distance_east: %.5f " % distance_east)

        # get new targetLocation
        current_location = vehicle.location.global_relative_frame
        target_location = get_location_metres(current_location, distance_north, distance_east)
        f.write("\n  target_location: %s " % target_location)

        # goto target location
        targetLocation = target_location
        currentLocation = vehicle.location.global_relative_frame
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        vehicle.simple_goto(targetLocation)
        time.sleep(5)

        f.write("\n DEBUG: targetLocation: %s" % targetLocation)
        f.write("\n DEBUG: targetLocation: %s" % targetDistance)






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
        f.write("\n DEBUG: mode: %s" % vehicle.mode.name)
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        f.write("\n Distance to target: %s " % remainingDistance)
        if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
            f.write("\n Reached target")
            f.write("Current location:  %s" % vehicle.location.local_frame)
            break;
        time.sleep(2)


# Set altitude to 5 meters above the current altitude
arm_and_takeoff(4)

f.write("\n Set groundspeed to 5m/s.")
vehicle.groundspeed=5



freq_880 = GPIO.input(16)
freq_1174 = GPIO.input(20)
freq_1318 = GPIO.input(21)


if freq_880:
    # Fly a path using specific GPS coordinates.
    f.write("\n Going to Position 1")
    point1 = LocationGlobalRelative(32.79144, -117.04690, 4)
    goto(point1)
    time.sleep(1)
    pixy_goto()

    # reduce altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = loc.alt - 3
    vehicle.simple_goto(loc)
    time.sleep(5)

    # drop payload by toggling GPIO pins
    GPIO.output(13, 0)
    GPIO.output(19, 1)
    time.sleep(1)

    # increase altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = 4
    vehicle.simple_goto(loc) # send command
    time.sleep(5)


elif freq_1174:
    f.write("\n Going to Position 2")
    point2 = LocationGlobalRelative(32.79135, -117.04699, 4)
    goto(point2)
    time.sleep(1)
    pixy_goto()

    # reduce altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = loc.alt - 3
    vehicle.simple_goto(loc)
    time.sleep(5)

    # drop payload by toggling GPIO pins
    GPIO.output(13, 1)
    GPIO.output(19, 0)
    time.sleep(1)

    # increase altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = 4
    vehicle.simple_goto(loc) # send command
    time.sleep(5)


elif freq_1318:
    f.write("\n Going to Position 3")
    point3 = LocationGlobalRelative(32.79144, -117.04714, 4)
    goto(point3)
    time.sleep(1)
    pixy_goto()

    # reduce altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = loc.alt - 3
    vehicle.simple_goto(loc)
    time.sleep(5)

    # drop payload by toggling GPIO pins
    GPIO.output(13, 0)
    GPIO.output(19, 0)
    time.sleep(1)

    # increase altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = 4
    vehicle.simple_goto(loc) # send command
    time.sleep(5)


else:
    # Fly a path using specific GPS coordinates.
    f.write("\n Going to Position 1")
    point1 = LocationGlobalRelative(32.79144, -117.04690, 4)
    goto(point1)
    time.sleep(1)
    pixy_goto()

    # reduce altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = loc.alt - 3
    vehicle.simple_goto(loc)
    time.sleep(5)

    # drop payload by toggling GPIO pins
    GPIO.output(13, 0)
    GPIO.output(19, 1)
    time.sleep(1)

    # increase altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = 4
    vehicle.simple_goto(loc) # send command
    time.sleep(5)

    f.write("\n Going to Position 2")
    point2 = LocationGlobalRelative(32.79135, -117.04699, 4)
    goto(point2)
    time.sleep(1)
    pixy_goto()

    # reduce altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = loc.alt - 3
    vehicle.simple_goto(loc)
    time.sleep(5)

    # drop payload by toggling GPIO pins
    GPIO.output(13, 1)
    GPIO.output(19, 0)
    time.sleep(1)

    # increase altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = 4
    vehicle.simple_goto(loc) # send command
    time.sleep(5)

    f.write("\n Going to Position 3")
    point3 = LocationGlobalRelative(32.79144, -117.04714, 4)
    goto(point3)
    time.sleep(1)
    pixy_goto()

    # reduce altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = loc.alt - 3
    vehicle.simple_goto(loc)
    time.sleep(5)

    # drop payload by toggling GPIO pins
    GPIO.output(13, 0)
    GPIO.output(19, 0)
    time.sleep(1)

    # increase altitude
    loc = vehicle.location.global_relative_frame #get current location
    loc.alt = 4
    vehicle.simple_goto(loc) # send command
    time.sleep(5)


f.write("\n Going home")
point4 = LocationGlobalRelative(vehicle.home_location.lat, vehicle.home_location.lon, 3)
goto(point4)

vehicle.mode = VehicleMode("LAND")
f.write("\n Completed")
