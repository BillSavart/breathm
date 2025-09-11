#!/usr/bin/env python

import sys
import time
from enum import Enum
import RPi.GPIO as GPIO
from bmp280 import BMP280
import numpy as np
from scipy.signal import butter, filtfilt

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
sampling_rate = 0.1 # 100ms 10Hz
lowpass_fs = 60.0
lowpass_cutoff = 2.0
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50

def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y

def init_guide_phase(pressures):
    filtered_pressures = lowpass_filter(pressures, lowpass_cutoff, lowpass_fs)
    state = UserState["INHALE"]
    filtered_breath_times = []
    
    count = 0
    
    print("===== BREATH TIMES =====")
    
    for i in range(1, len(filtered_pressures)):
        print("Pressure: ", pressures[i-1])
        print("Filtered pressure: ", filtered_pressures[i-1])
        if filtered_pressures[i] < filtered_pressures[i - 1]:
            if state == UserState["INHALE"]:
                state = UserState["EXHALE"]
        elif filtered_pressures[i] > filtered_pressures[i - 1]:
            if state == UserState["EXHALE"]:
                state = UserState["INHALE"]
                breath_time = float(count) * float(sampling_rate)
                print("Breath time: ", breath_time.__round__(2))
                filtered_breath_times.append(breath_time.__round__(2))
                count = 0

        count += 1

    # use median as the breath time
    target_breath_time = np.median(filtered_breath_times)

    return target_breath_time

def real_time_lowpass_filter(pressures):
    filtered_pressures = lowpass_filter(pressures, lowpass_cutoff, lowpass_fs)
    return filtered_pressures[-1]

def validate_stable(pressures, target_breath_time, validate_count, remove_count):
    filtered_pressures = lowpass_filter(pressures, lowpass_cutoff, lowpass_fs)
    state = UserState["INHALE"]
    filtered_breath_times = []
    
    count = 0
    pressure_index = [0]
    eval_state = EvalState["NONE"]
    next_target_breath_time = target_breath_time

    print("===== BREATH TIMES =====")

    for i in range(1, len(filtered_pressures)):
        if filtered_pressures[i] < filtered_pressures[i - 1]:
            if state == UserState["INHALE"]:
                state = UserState["EXHALE"]
        elif filtered_pressures[i] > filtered_pressures[i - 1]:
            if state == UserState["EXHALE"]:
                state = UserState["INHALE"]
                pressure_index.append(i)
                breath_time = float(count) * float(sampling_rate)
                print("Breath time: ", breath_time.__round__(2))
                filtered_breath_times.append(breath_time.__round__(2))
                count = 0 

        count += 1

    len_breath = len(filtered_breath_times) + remove_count - validate_count

    if (len_breath < sampling_window):
        return eval_state, validate_count, remove_count, pressures, next_target_breath_time
    
    else:
        start_idx = validate_count - remove_count
        end_idx = start_idx + sampling_window
        filtered_breath_times = filtered_breath_times[start_idx:end_idx]
        cutoff_index = pressure_index[start_idx]
        pressures = pressures[cutoff_index:]
        remove_count = validate_count
        validate_count += 1

    percentage_deviation = np.abs((np.asarray(filtered_breath_times) - target_breath_time) / target_breath_time) * 100


    if np.all(percentage_deviation <= success_threshold):
        eval_state = EvalState["SUCCESS"]
        next_target_breath_time = target_breath_time + increase_breath_time
    elif np.any(percentage_deviation > fail_threshold):
        next_target_breath_time = target_breath_time - increase_breath_time / 2
        eval_state = EvalState["FAIL"]

    return eval_state, validate_count, remove_count, pressures, next_target_breath_time

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

def mirror_breathing(curr_pressure, prev_pressure, position, direction, vibration_pwm):
    if (prev_pressure != 0 and curr_pressure > prev_pressure): # inhale
        if (vibrate_duty_cycle != 30):
            vibrate_duty_cycle = 30
            vibration_pwm.ChangeDutyCycle(vibrate_duty_cycle)

        if (position <= linear_actuator_max_distance):
            direction = 1

        else:
            direction = 0

    elif (prev_pressure != 0 and curr_pressure < prev_pressure): # exhale
        if (vibrate_duty_cycle != 0):
            vibrate_duty_cycle = 0
            vibration_pwm.ChangeDutyCycle(vibrate_duty_cycle)

        if (position >= 0):
            direction = -1

        else:
            direction = 0

    move_linear_actuator(direction)

    position += direction

    return position, direction

def guide_breathing(machine_breath, target_breath, position):
    direction = 0
    if (machine_breath < target_breath / 2):
        if (position <= linear_actuator_max_distance):
            direction = 1

        else:
            direction = 0

        machine_breath += sampling_rate
    
    elif (machine_breath >= target_breath / 2 and machine_breath < target_breath):
        if (position >= 0):
            direction = -1

        else:
            direction = 0

        machine_breath += sampling_rate

    move_linear_actuator(direction)
    position += direction

    if (machine_breath >= target_breath):
        machine_breath = sampling_rate

    return machine_breath, position

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
    pressures = []
    prev_filtered_pressure = 0

    user_state = -1
    eval_state = EvalState["NONE"]

    # target
    target_breath_time = 3 #DEBUG

    # machine state
    machine_state = MachineState["GUIDE"] #DEBUG
    machine_breath = 0
    curr_state_count = 0
    guide_window_reach = False

    validate_count = 0
    remove_count = 0

    # linear actuator
    la_position = 0
    la_direction = 0

    while True:
        curr_pressure = bmp280.get_pressure()
        pressures.append(curr_pressure)

        if (machine_state == MachineState["MIRROR"]):
            curr_state_count += sampling_rate
            if (len(pressures) > 1):
                la_position, la_direction = mirror_breathing(curr_pressure, pressures[-2], la_position, la_direction, vibration_pwm)

        elif (machine_state == MachineState["GUIDE"]):
            curr_state_count += sampling_rate
            # curr_pressure = real_time_lowpass_filter(pressures)
            # pressures.pop(0)
            machine_breath, la_position = guide_breathing(machine_breath, target_breath_time, la_position)

        if ((machine_state == MachineState["MIRROR"] and len(pressures) > 1 and curr_pressure > pressures[-2]) or 
            (machine_state == MachineState["GUIDE"] and curr_pressure > prev_filtered_pressure)): # inhale
            if (user_state != UserState["INHALE"]):
                # TRANSITION FROM EXHALE TO INHALE
                # USER FINISH 1 BREATH TIME

                if (machine_state == MachineState["MIRROR"] and curr_state_count >= 60):
                    # INITIALIZE GUIDE PHASE
                    target_breath_time = init_guide_phase(pressures)
                    print("Target breath time: ", target_breath_time)

                    machine_state = MachineState["GUIDE"]
                    curr_state_count = 0
                    pressures = []

                    vibrate_duty_cycle = 0
                    vibration_pwm.ChangeDutyCycle(vibrate_duty_cycle)

                elif (machine_state == MachineState["GUIDE"]):
                    if (guide_window_reach == False and curr_state_count >= target_breath_time * sampling_window): # and curr_state_count > 30
                        # AFTER 4 INITIAL GUIDE
                        eval_state, validate_count, remove_count, pressures, target_breath_time  = validate_stable(pressures, target_breath_time, validate_count, remove_count)
                        curr_state_count = 0
                        guide_window_reach = True

                    elif (guide_window_reach == True and curr_state_count >= target_breath_time):
                        # AFTER 1 GUIDE
                        eval_state, validate_count, remove_count, pressures, target_breath_time = validate_stable(pressures, target_breath_time, validate_count, remove_count)
                        curr_state_count = 0

                    if (eval_state == EvalState["FAIL"] or eval_state == EvalState["SUCCESS"]):
                        print("Eval state: ", eval_state == EvalState["SUCCESS"])
                        guide_window_reach = False
                        eval_state = EvalState["NONE"]
                        validate_count = 0
                        remove_count = 0
                        pressures = []
                        print("Target breath time: ", target_breath_time)

                # RESET STATE
                user_state = UserState["INHALE"]

        elif ((machine_state == MachineState["MIRROR"] and len(pressures) > 1 and curr_pressure < pressures[-2]) or 
            (machine_state == MachineState["GUIDE"] and curr_pressure < prev_filtered_pressure)): # exhale
            if (user_state != UserState["EXHALE"]):
                # TRANSITION FROM INHALE TO EXHALE
                user_state = UserState["EXHALE"]

        if (machine_state == MachineState["GUIDE"]):
            prev_filtered_pressure = curr_pressure
            
        time.sleep(sampling_rate)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        GPIO.cleanup()
        sys.exit(0)
