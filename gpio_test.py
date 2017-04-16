#!/usr/bin/env python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

GPIO.output(13, GPIO.LOW)
GPIO.output(19, GPIO.LOW)

# drop payload by toggling GPIO pins
print("\n Drop GPIO 1!")
GPIO.output(13, GPIO.LOW)
GPIO.output(19, GPIO.HIGH)
time.sleep(3)

# drop payload by toggling GPIO pins
print("\n Drop GPIO 2!")
GPIO.output(13, GPIO.HIGH)
GPIO.output(19, GPIO.LOW)
time.sleep(3)

# drop payload by toggling GPIO pins
print("\n Drop GPIO 3!")
GPIO.output(13, GPIO.HIGH)
GPIO.output(19, GPIO.HIGH)
time.sleep(3)


print("Completed")
GPIO.cleanup()
