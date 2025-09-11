#!/usr/bin/env python

import sys
import time
from enum import Enum
import RPi.GPIO as GPIO
from bmp280 import BMP280
import numpy as np
from scipy.signal import butter, filtfilt
from datetime import datetime

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

class MachineState (Enum):
    MIRROR = 0
    GUIDE = 1

class UserState (Enum):
    INHALE = 0
    EXHALE = 1

class EvalState (Enum):
    NONE = 0
    FAIL = 1
    SUCCESS = 2

# pin position
in1 = 23
in2 = 24
en = 25
vibration_pin = 27
vibrate_duty_cycle = 0

# user condition
user_inhale_time = 0
user_exhale_time = 0

# predefined variables
sampling_rate = 0.1 # 20ms 50Hz
lowpass_fs = 60.0
lowpass_cutoff = 2.0
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50

def move_linear_actuator(direction):
    if (direction == 1):
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)

    elif (direction == -1):
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2 ,GPIO.LOW)

def mirror_breathing(curr_pressure, prev_pressure, position, direction):
    if (prev_pressure != 0 and curr_pressure > prev_pressure): # inhale
        if (position <= linear_actuator_max_distance):
            direction = 1

        else:
            direction = 0

    elif (prev_pressure != 0 and curr_pressure < prev_pressure): # exhale
        if (position >= 0):
            direction = -1

        else:
            direction = 0

    move_linear_actuator(direction)

    position += direction

    return position, direction

def main():
    GPIO.setmode(GPIO.BCM)

    # init linear actuators
    GPIO.setup(in1,GPIO.OUT)
    GPIO.setup(in2,GPIO.OUT)
    GPIO.setup(en,GPIO.OUT)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    p = GPIO.PWM(en,800)
    p.start(100)
    
    # init bmp280
    bus = SMBus(1)
    bmp280 = BMP280(i2c_dev=bus)
    bmp280.setup(mode="forced")

    # init vibration pin
    GPIO.setup(vibration_pin, GPIO.OUT)
    vibration_pwm = GPIO.PWM(vibration_pin, 50)
    vibration_pwm.start(0)

    # user state
    curr_pressure = 0
    prev_pressure = 0
    
    la_position = 0
    la_direction = 0

    while True:
        curr_pressure = bmp280.get_pressure()
        print(curr_pressure)
        la_position, la_direction = mirror_breathing(curr_pressure, prev_pressure, la_position, la_direction)
        prev_pressure = curr_pressure
        time.sleep(sampling_rate)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        GPIO.cleanup()
        sys.exit(0)
