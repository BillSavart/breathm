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
vibration_pin = 27
vibrate_duty_cycle = 0

# --- Parameters from Thesis  ---
# Thesis specified Sampling Frequency fs = 60 Hz
sampling_rate = 1.0 / 60.0  # Approx 0.0167 seconds
lowpass_fs = 60.0           # Matches the actual execution speed
lowpass_cutoff = 2.0        # Cut-off frequency 2Hz 
lowpass_order = 4           # Degree 4 

# Other parameters
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50

# --- Real-time Filter Class (The Root Cause Fix) ---
class RealTimeFilter:
    def __init__(self, order, cutoff, fs):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        # Get filter coefficients (b, a)
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        # Initialize filter state (zi) for real-time processing
        self.zi = lfilter_zi(self.b, self.a)
    
    def process(self, value):
        # Filter one sample at a time, updating the state
        filtered_value, self.zi = lfilter(self.b, self.a, [value], zi=self.zi)
        return filtered_value[0]

# --- Helper Functions ---

def init_guide_phase(breath_times):
    # According to Thesis Algorithm 2: Use Median of collected breath times [cite: 131, 133]
    # Note: The original code re-calculated breath times from pressure history.
    # To simplify and be robust, we can use the breath times detected during Mirror phase.
    if not breath_times:
        return 3.0 # Fallback
    
    target_breath_time = np.median(breath_times)
    print(f"Init Guide Phase: Median Breath Time = {target_breath_time:.2f}")
    return target_breath_time

def validate_stable(breath_times, target_breath_time):
    # Thesis Algorithm 3 [cite: 136]
    # Calculate percentage deviation
    
    if len(breath_times) < sampling_window:
        return EvalState.NONE, target_breath_time

    # Use the last 'sampling_window' breaths
    recent_breaths = np.array(breath_times[-sampling_window:])
    
    deviations = ((recent_breaths - target_breath_time) / target_breath_time) * 100
    
    print(f"Validation Deviations: {deviations}")

    next_target = target_breath_time
    state = EvalState.NONE

    # "If all percentage deviations are larger than -10%" (Success) [cite: 141]
    # Note: Thesis says > -10%, code used abs <= 15. Let's stick to user's success threshold logic or thesis?
    # Let's align with Thesis logic: Success implies user is slower or matching (deviation > -10%)
    # But usually, we want them close. The original code's abs logic is safer for "matching".
    # I will keep the original code's logic as it's a practical implementation of "following".
    if np.all(np.abs(deviations) <= success_threshold):
        state = EvalState.SUCCESS
        next_target = target_breath_time + increase_breath_time # Slower [cite: 142]
    
    # "If all percentage deviations are greater than 50%" (Fail - too slow/fast discrepancy) [cite: 143]
    elif np.any(np.abs(deviations) > fail_threshold):
        state = EvalState.FAIL
        # Thesis says: "adjusted to match user's current breath time" [cite: 143]
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

def mirror_breathing_logic(curr_filtered, prev_filtered, position, direction, vibration_pwm):
    # Thesis Algorithm 1: Compare current and previous pressure [cite: 129, 133]
    # Now using FILTERED data, so it won't jitter.
    
    # Simple Hysteresis can be added here if 60Hz is still too sensitive, 
    # but the Low-Pass filter should handle most noise.
    
    is_inhaling = curr_filtered > prev_filtered
    is_exhaling = curr_filtered < prev_filtered

    if is_inhaling: # Inhale
        # Thesis: "vibrates during inhalation" [cite: 130]
        if vibrate_duty_cycle != 30:
            global vibrate_duty_cycle
            vibrate_duty_cycle = 30
            vibration_pwm.ChangeDutyCycle(vibrate_duty_cycle)

        if position <= linear_actuator_max_distance:
            direction = 1
        else:
            direction = 0

    elif is_exhaling: # Exhale
        if vibrate_duty_cycle != 0:
            vibrate_duty_cycle = 0
            vibration_pwm.ChangeDutyCycle(vibrate_duty_cycle)

        if position >= 0:
            direction = -1
        else:
            direction = 0

    move_linear_actuator(direction)
    position += direction
    return position, direction, (UserState.INHALE if is_inhaling else UserState.EXHALE)

def guide_breathing_logic(machine_breath, target_breath, position):
    # Thesis 5.2: Device gives feedback to gradually slow down [cite: 134]
    direction = 0
    half_cycle = target_breath / 2.0
    
    if machine_breath < half_cycle: # Inhale phase of Guide
        if position <= linear_actuator_max_distance:
            direction = 1
        else:
            direction = 0
        machine_breath += sampling_rate
    
    elif machine_breath >= half_cycle and machine_breath < target_breath: # Exhale phase
        if position >= 0:
            direction = -1
        else:
            direction = 0
        machine_breath += sampling_rate

    if machine_breath >= target_breath:
        machine_breath = 0 # Reset cycle

    move_linear_actuator(direction)
    position += direction

    return machine_breath, position

def main():
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

    GPIO.setup(vibration_pin, GPIO.OUT)
    vibration_pwm = GPIO.PWM(vibration_pin, 50)
    vibration_pwm.start(0)

    # --- Initialize Real-Time Filter ---
    # Fixes root cause: No more raw data processing
    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs)

    # State Variables
    curr_pressure = 0
    prev_filtered_pressure = 0
    
    # Start in MIRROR mode as per thesis flow (Figure 5.1) [cite: 128]
    machine_state = MachineState.MIRROR 
    print("State: MIRROR Phase Started")

    user_state = UserState.EXHALE # Assume start with exhale or wait for first inhale
    curr_state_timer = 0
    
    # Data Collection
    detected_breath_times = []
    current_breath_duration = 0
    
    # Actuator State
    la_position = 0
    la_direction = 0
    
    # Guide State
    target_breath_time = 3.0
    machine_breath_timer = 0
    
    # Initial pressure read to prime the filter
    first_read = bmp280.get_pressure()
    prev_filtered_pressure = rt_filter.process(first_read)

    try:
        while True:
            start_time = time.time()
            
            # 1. Sensing & Filtering
            raw_pressure = bmp280.get_pressure()
            curr_filtered_pressure = rt_filter.process(raw_pressure)
            
            # 2. Detect User Breath Phase (Crossing points)
            # Simple peak detection based on slope change
            user_action = None
            if curr_filtered_pressure > prev_filtered_pressure:
                user_action = UserState.INHALE
            elif curr_filtered_pressure < prev_filtered_pressure:
                user_action = UserState.EXHALE
            
            # Detect Breath Cycle Completion (Transition from Exhale to Inhale)
            if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                # Cycle complete
                if current_breath_duration > 0.5: # Filter out glitches < 0.5s
                    detected_breath_times.append(current_breath_duration)
                    print(f"Breath Detected: {current_breath_duration:.2f}s")
                current_breath_duration = 0
                user_state = UserState.INHALE
            elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                user_state = UserState.EXHALE
            
            current_breath_duration += sampling_rate
            curr_state_timer += sampling_rate

            # 3. State Machine Logic
            if machine_state == MachineState.MIRROR:
                # Actuate based on real-time filtered data
                la_position, la_direction, _ = mirror_breathing_logic(
                    curr_filtered_pressure, prev_filtered_pressure, 
                    la_position, la_direction, vibration_pwm
                )
                
                # Check for Phase Transition (1 minute) [cite: 131]
                if curr_state_timer >= 60.0:
                    print("--- Switching to GUIDE Phase ---")
                    target_breath_time = init_guide_phase(detected_breath_times)
                    machine_state = MachineState.GUIDE
                    curr_state_timer = 0
                    detected_breath_times = [] # Clear for evaluation
                    
                    # Turn off vibration for Guide phase (as implied by thesis 4.2 distinguishing phases) [cite: 119]
                    vibration_pwm.ChangeDutyCycle(0)

            elif machine_state == MachineState.GUIDE:
                # Actuate based on Target Breath Time
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # Check for Evaluation Window (every 4 breaths or roughly target * 4)
                # Thesis implies evaluation window of size 4 [cite: 136]
                if len(detected_breath_times) >= sampling_window:
                    eval_state, new_target = validate_stable(detected_breath_times, target_breath_time)
                    
                    if eval_state == EvalState.SUCCESS:
                        print("User Sync Success! Slowing down...")
                        target_breath_time = new_target
                        detected_breath_times = [] # Reset window [cite: 145]
                    elif eval_state == EvalState.FAIL:
                        print("User Sync Fail. Adjusting to user pace...")
                        target_breath_time = new_target
                        detected_breath_times = [] # Reset window [cite: 145]
                    else:
                        # Slide window (remove oldest)
                        detected_breath_times.pop(0)

            # Update history
            prev_filtered_pressure = curr_filtered_pressure
            
            # Loop Timing Control (Maintain 60Hz)
            elapsed = time.time() - start_time
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main()