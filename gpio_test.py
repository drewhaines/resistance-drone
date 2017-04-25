#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

GPIO.output(13, GPIO.LOW)
GPIO.output(19, GPIO.LOW)



print("Completed")
GPIO.cleanup()
