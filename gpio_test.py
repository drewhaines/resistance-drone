#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

GPIO.output(17, GPIO.LOW)
GPIO.output(27, GPIO.LOW)

# drop payload by toggling GPIO pins
print("\n Drop GPIO 1!")
GPIO.output(17, GPIO.LOW)
GPIO.output(27, GPIO.HIGH)
print("\n GPIO 17 %3d" % GPIO.input(17))
print("\n GPIO 17 %3d" % GPIO.input(27))

time.sleep(3)

# drop payload by toggling GPIO pins
print("\n Drop GPIO 2!")
GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.LOW)
print("\n GPIO 17 %3d" % GPIO.input(17))
print("\n GPIO 17 %3d" % GPIO.input(27))
time.sleep(3)

# drop payload by toggling GPIO pins
print("\n Drop GPIO 3!")
GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.HIGH)
print("\n GPIO 17 %3d" % GPIO.input(17))
print("\n GPIO 17 %3d" % GPIO.input(27))
time.sleep(3)


print("Completed")
GPIO.cleanup()
