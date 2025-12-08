#!/usr/bin/env python

import sys
import time
from enum import Enum
import RPi.GPIO as GPIO
from bmp280 import BMP280
import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

class MachineState(Enum):
    WARMUP = -1  
    MIRROR = 0
    GUIDE = 1

class UserState(Enum):
    INHALE = 0
    EXHALE = 1

class EvalState(Enum):
    NONE = 0
    FAIL = 1
    SUCCESS = 2

# --- Pin Definition ---
in1 = 23
in2 = 24
en = 25

# --- Parameters ---
sampling_rate = 1.0 / 60.0  
lowpass_fs = 60.0           
lowpass_cutoff = 2.0        
lowpass_order = 4           

# Other parameters
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50
warmup_duration = 5.0  

# --- Real-time Filter Class ---
class RealTimeFilter:
    def __init__(self, order, cutoff, fs, initial_value=0.0):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.zi = lfilter_zi(self.b, self.a) * initial_value
    
    def process(self, value):
        filtered_value, self.zi = lfilter(self.b, self.a, [value], zi=self.zi)
        return filtered_value[0]

# --- Helper Functions ---

def init_guide_phase(breath_times):
    if not breath_times:
        print("Warning: No breath data collected. Defaulting to 3.0s")
        return 3.0 
    target_breath_time = np.median(breath_times)
    print(f"Init Guide Phase: Median Breath Time calculated as {target_breath_time:.2f}s")
    return target_breath_time

def validate_stable(breath_times, target_breath_time):
    if len(breath_times) < sampling_window:
        return EvalState.NONE, target_breath_time

    recent_breaths = np.array(breath_times[-sampling_window:])
    deviations = ((recent_breaths - target_breath_time) / target_breath_time) * 100
    
    next_target = target_breath_time
    state = EvalState.NONE

    if np.all(np.abs(deviations) <= success_threshold):
        state = EvalState.SUCCESS
        next_target = target_breath_time + increase_breath_time
    elif np.any(np.abs(deviations) > fail_threshold):
        state = EvalState.FAIL
        next_target = np.mean(recent_breaths) 
        
    return state, next_target

def move_linear_actuator(direction):
    if direction == 1:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == -1:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

def guide_breathing_logic(machine_breath, target_breath, position):
    direction = 0
    half_cycle = target_breath / 2.0
    
    if machine_breath < half_cycle: 
        if position <= linear_actuator_max_distance:
            direction = 1
        else:
            direction = 0
        machine_breath += sampling_rate
    
    elif machine_breath >= half_cycle and machine_breath < target_breath: 
        if position >= 0:
            direction = -1
        else:
            direction = 0
        machine_breath += sampling_rate

    if machine_breath >= target_breath:
        machine_breath = 0 

    move_linear_actuator(direction)
    position += direction

    return machine_breath, position

def main():
    print("Program Starting...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(en, GPIO.OUT)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    p = GPIO.PWM(en, 800)
    p.start(100)
    
    bus = SMBus(1)
    bmp280 = BMP280(i2c_dev=bus)
    bmp280.setup(mode="forced")

    first_read = bmp280.get_pressure()
    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)

    # State Variables
    machine_state = MachineState.WARMUP 
    print(f">>> System Warmup ({warmup_duration}s)... Please breathe naturally.")

    user_state = UserState.EXHALE
    program_start_time = time.time()
    phase_start_time = 0 
    
    # Data Collection
    detected_breath_times = []
    current_breath_duration = 0
    skip_first_breath = True

    # Actuator State
    la_position = 0
    
    # Guide State
    target_breath_time = 3.0
    machine_breath_timer = 0
    
    # Prime filter
    prev_filtered_pressure = rt_filter.process(first_read)

    try:
        while True:
            loop_start = time.time()
            
            # 1. Sensing & Filtering
            raw_pressure = bmp280.get_pressure()
            curr_filtered_pressure = rt_filter.process(raw_pressure)
            
            # 2. Detect User Breath Phase
            user_action = None
            if curr_filtered_pressure > prev_filtered_pressure:
                user_action = UserState.INHALE
            elif curr_filtered_pressure < prev_filtered_pressure:
                user_action = UserState.EXHALE
            
            # --- WARMUP PHASE ---
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0) # Motor OFF
                
                if user_action is not None:
                    user_state = user_action

                if time.time() - program_start_time >= warmup_duration:
                    print("\n" + "="*40)
                    print(">>> MIRROR Phase (Sensing Only) Started (60s)")
                    print(">>> Motor will remain STILL to collect accurate data.")
                    print("="*40 + "\n")
                    
                    machine_state = MachineState.MIRROR
                    phase_start_time = time.time()
                    current_breath_duration = 0
                    skip_first_breath = True
                    detected_breath_times = []

            # --- MIRROR PHASE (SILENT / CALIBRATION) ---
            elif machine_state == MachineState.MIRROR:
                # [MODIFIED] Motor stays OFF during Mirror phase
                move_linear_actuator(0) 

                # Data Collection Logic (Still Active)
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        if skip_first_breath:
                            skip_first_breath = False
                        else:
                            detected_breath_times.append(current_breath_duration)
                            # print(f"Detected: {current_breath_duration:.2f}s") # Debug
                    
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate
                
                # Check for Phase Transition (60s)
                if time.time() - phase_start_time >= 60.0:
                    print("\n" + "="*30)
                    print(">>> TIMEOUT: Switching to GUIDE Phase")
                    print(">>> Motor Starting...")
                    print("="*30 + "\n")
                    
                    target_breath_time = init_guide_phase(detected_breath_times)
                    machine_state = MachineState.GUIDE
                    detected_breath_times = [] 

            # --- GUIDE PHASE (MOTOR ON) ---
            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # Evaluation Logic
                if len(detected_breath_times) >= sampling_window:
                    eval_state, new_target = validate_stable(detected_breath_times, target_breath_time)
                    
                    if eval_state == EvalState.SUCCESS:
                        print(f"Evaluation: SUCCESS! Slowing down to {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    elif eval_state == EvalState.FAIL:
                        print(f"Evaluation: FAIL. Adjusting back to {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    else:
                        detected_breath_times.pop(0)

                # Background User Detection
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        detected_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                current_breath_duration += sampling_rate

            prev_filtered_pressure = curr_filtered_pressure
            
            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt")
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main()