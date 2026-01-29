#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from enum import Enum
from scipy.signal import butter, lfilter, lfilter_zi

# --- GPIO & Sensor ---
try:
    import RPi.GPIO as GPIO
    from bmp280 import BMP280
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

in1, in2, en = 23, 24, 25
sampling_rate = 1.0 / 60.0  
lowpass_fs = 60.0           
lowpass_cutoff = 2.0        
lowpass_order = 4           
linear_actuator_max_distance = 50
warmup_duration = 3.0
mirror_duration = 30.0

class RealTimeFilter:
    def __init__(self, order, cutoff, fs, initial_value=0.0):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.zi = lfilter_zi(self.b, self.a) * initial_value
    def process(self, value):
        filtered_value, self.zi = lfilter(self.b, self.a, [value], zi=self.zi)
        return filtered_value[0]

def move_linear_actuator(direction):
    try:
        if direction == 1:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif direction == -1:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
    except: pass

def guide_breathing_logic(timer, target, pos):
    direct = 0
    half = target / 2.0
    action = ""
    if timer < half:
        if pos <= linear_actuator_max_distance: direct = 1
        else: direct = 0
        timer += sampling_rate
        action = "INHALE"
    elif timer >= half and timer < target:
        if pos >= 0: direct = -1
        else: direct = 0
        timer += sampling_rate
        action = "EXHALE"
    if timer >= target: timer = 0
    move_linear_actuator(direct)
    pos += direct
    return timer, pos, action

def main(stop_event=None, msg_callback=None):
    print(">>> [Mirror Version] 啟動 (無圖表)")
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(en, GPIO.OUT)
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        p = GPIO.PWM(en, 800)
        p.start(100)
    except: return

    try:
        bus = SMBus(1)
        bmp280 = BMP280(i2c_dev=bus)
        bmp280.setup(mode="forced")
        first_read = bmp280.get_pressure()
    except:
        p.stop()
        GPIO.cleanup()
        return

    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)
    machine_state = MachineState.WARMUP
    user_state = UserState.EXHALE
    
    program_start_time = time.time()
    mirror_start_time = 0
    mirror_breath_times = []
    current_breath_duration = 0
    la_position = 0
    target_breath_time = 3.0
    machine_breath_timer = 0
    prev_filtered = rt_filter.process(first_read)
    last_sent_action = ""

    try:
        while not (stop_event and stop_event.is_set()):
            loop_start = time.time()
            raw = bmp280.get_pressure()
            curr_filtered = rt_filter.process(raw)
            
            user_action = None
            if curr_filtered > prev_filtered: user_action = UserState.INHALE
            elif curr_filtered < prev_filtered: user_action = UserState.EXHALE

            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0)
                if user_action is not None: user_state = user_action
                if time.time() - program_start_time >= warmup_duration:
                    machine_state = MachineState.MIRROR
                    mirror_start_time = time.time()
                    print(">>> MIRROR 模式 (偵測中)")

            elif machine_state == MachineState.MIRROR:
                move_linear_actuator(0)
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.8: mirror_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                current_breath_duration += sampling_rate

                if time.time() - mirror_start_time >= mirror_duration:
                    if len(mirror_breath_times) > 0: target_breath_time = np.mean(mirror_breath_times)
                    else: target_breath_time = 4.0
                    machine_state = MachineState.GUIDE
                    print(f">>> GUIDE 模式 (Target: {target_breath_time:.2f}s)")

            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position, action = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # 發送訊號
                if msg_callback and action != last_sent_action:
                    msg_callback(f"ANIM:{action}\n")
                    last_sent_action = action

            prev_filtered = curr_filtered
            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0: time.sleep(sleep_time)

    finally:
        p.stop()
        GPIO.cleanup()

# [重點] 補上啟動指令
if __name__ == "__main__":
    main()