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
import time

time.sleep(3)

freq_880 = GPIO.input(16)
freq_1174 = GPIO.input(20)
freq_1318 = GPIO.input(21)


if freq_880:
    f.write("\n 800 Hz is on!")

elif freq_1174:
    f.write("\n 1174 Hz is on!")

elif freq_1318:
    f.write("\n 1318 Hz is on!")

else:
    f.write("\n No frequencies are on")


f.write("Completed")
GPIO.cleanup()
