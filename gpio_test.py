#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

GPIO.output(13, GPIO.LOW)
GPIO.output(19, GPIO.LOW)

while 1:
    # drop payload by toggling GPIO pins
    print("\n Drop GPIO!")
    GPIO.output(13, 1)
    GPIO.output(19, 1)
    print("\n GPIO 17 %3d" % GPIO.input(13))
    print("\n GPIO 27 %3d" % GPIO.input(19))
    time.sleep(3)


print("Completed")
GPIO.cleanup()
