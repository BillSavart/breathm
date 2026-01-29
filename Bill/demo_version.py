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
    GUIDE = 1 

# Pin Definition
in1, in2, en = 23, 24, 25
sampling_rate = 1.0 / 60.0  
linear_actuator_max_distance = 50
warmup_duration = 3.0

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

# --- Main Logic ---
def main(stop_event=None, msg_callback=None):
    print(">>> [Demo Version] 啟動 (無圖表)")
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

    machine_state = MachineState.WARMUP
    program_start_time = time.time()
    la_position = 0
    target_breath_time = 3.0
    machine_breath_timer = 0
    last_sent_action = ""

    try:
        while not (stop_event and stop_event.is_set()):
            loop_start = time.time()
            
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0)
                if time.time() - program_start_time >= warmup_duration:
                    print(">>> 暖機完成 -> GUIDE")
                    machine_state = MachineState.GUIDE

            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position, action = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # 發送訊號
                if msg_callback and action != last_sent_action:
                    msg_callback(f"ANIM:{action}\n")
                    last_sent_action = action

            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0: time.sleep(sleep_time)

    finally:
        p.stop()
        GPIO.cleanup()

# [重點] 補上啟動指令
if __name__ == "__main__":
    main()