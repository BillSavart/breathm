#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import threading
import collections
from enum import Enum
import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi

# --- Matplotlib 設定 ---
import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- GPIO & Sensor Imports ---
try:
    import RPi.GPIO as GPIO
    from bmp280 import BMP280
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus 
    print("Warning: SMBus or BMP280 library mismatch.")

# --- 狀態定義 ---
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

# --- Global Shared Data ---
MAX_POINTS = 600
data_lock = threading.Lock()
pressure_data = collections.deque(maxlen=MAX_POINTS)
position_data = collections.deque(maxlen=MAX_POINTS)
time_data = collections.deque(maxlen=MAX_POINTS)
current_mode_text = "Initializing"
running = True 

# --- Pin Definition ---
in1, in2, en = 23, 24, 25

# --- Parameters ---
sampling_rate = 1.0 / 60.0  
lowpass_fs, lowpass_cutoff, lowpass_order = 60.0, 2.0, 4
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold, fail_threshold = 15, 50

warmup_duration = 5.0
mirror_duration = 60.0

# --- Filter Class ---
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
def validate_stable(breath_times, target_breath_time):
    if len(breath_times) < sampling_window:
        return EvalState.NONE, target_breath_time
    recent = np.array(breath_times[-sampling_window:])
    deviations = ((recent - target_breath_time) / target_breath_time) * 100
    if np.all(np.abs(deviations) <= success_threshold):
        return EvalState.SUCCESS, target_breath_time + increase_breath_time
    elif np.any(np.abs(deviations) > fail_threshold):
        return EvalState.FAIL, np.mean(recent) 
    return EvalState.NONE, target_breath_time

def move_linear_actuator(direction):
    if not running: return
    try:
        if direction == 1:
            GPIO.output(in1, GPIO.HIGH); GPIO.output(in2, GPIO.LOW)
        elif direction == -1:
            GPIO.output(in1, GPIO.LOW); GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW); GPIO.output(in2, GPIO.LOW)
    except: pass

def guide_breathing_logic(timer, target, pos):
    direct = 0
    half = target / 2.0
    if timer < half:
        direct = 1 if pos <= linear_actuator_max_distance else 0
    elif timer < target:
        direct = -1 if pos >= 0 else 0
    timer += sampling_rate
    if timer >= target: timer = 0
    move_linear_actuator(direct)
    pos += direct
    return timer, pos

def control_loop():
    global running, current_mode_text
    GPIO.setmode(GPIO.BCM)
    for pin in [in1, in2, en]: GPIO.setup(pin, GPIO.OUT)
    p = GPIO.PWM(en, 800); p.start(100)
    
    try:
        bus = SMBus(1); bmp280 = BMP280(i2c_dev=bus)
        bmp280.setup(mode="forced")
        first_read = bmp280.get_pressure()
    except Exception as e:
        print(f"Sensor Error: {e}"); running = False; return

    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)
    machine_state, user_state = MachineState.WARMUP, UserState.EXHALE
    current_mode_text = "WARMUP"
    
    program_start_time = time.time()
    mirror_start_time = 0
    mirror_breath_times, detected_breath_times = [], []
    current_breath_duration, la_position = 0, 0
    target_breath_time, machine_breath_timer = 3.0, 0
    prev_filtered = rt_filter.process(first_read)

    try:
        while running:
            loop_start = time.time()
            raw = bmp280.get_pressure()
            curr_filtered = rt_filter.process(raw)
            
            # 判斷使用者動作
            user_action = UserState.INHALE if curr_filtered > prev_filtered else UserState.EXHALE

            # --- 狀態機 (繼承自 fix_version.py 邏輯) ---
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0)
                if time.time() - program_start_time >= warmup_duration:
                    machine_state = MachineState.MIRROR
                    current_mode_text = "MIRRORING (60s)"
                    mirror_start_time = time.time()
                    current_breath_duration = 0

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
                    # 關鍵：計算初始平均頻率
                    target_breath_time = np.mean(mirror_breath_times) if mirror_breath_times else 4.0
                    machine_state = MachineState.GUIDE
                    current_mode_text = f"GUIDE (Initial: {target_breath_time:.1f}s)"
                    current_breath_duration, machine_breath_timer = 0, 0

            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position = guide_breathing_logic(machine_breath_timer, target_breath_time, la_position)
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.5: detected_breath_times.append(current_breath_duration)
                    current_breath_duration, user_state = 0, UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                current_breath_duration += sampling_rate

                if len(detected_breath_times) >= sampling_window:
                    eval_st, new_target = validate_stable(detected_breath_times, target_breath_time)
                    if eval_st != EvalState.NONE:
                        target_breath_time = new_target
                        current_mode_text = f"GUIDE (Target: {target_breath_time:.1f}s)"
                        detected_breath_times = []
                    else:
                        detected_breath_times.pop(0)

            prev_filtered = curr_filtered
            with data_lock:
                pressure_data.append(curr_filtered); position_data.append(la_position); time_data.append(time.time() - program_start_time)
            
            sleep_time = sampling_rate - (time.time() - loop_start)
            if sleep_time > 0: time.sleep(sleep_time)

    finally:
        p.stop(); GPIO.cleanup(); running = False

def main():
    global running
    t = threading.Thread(target=control_loop); t.daemon = True; t.start()
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    ax1.get_yaxis().get_major_formatter().set_useOffset(False)
    line_p, = ax1.plot([], [], 'b-', lw=2); line_m, = ax2.plot([], [], 'r-', lw=2)
    ax1.set_xlim(0, 10); ax2.set_ylim(-5, 60)
    ax2.set_ylabel("Motor Pos"); ax2.set_xlabel("Time (s)")

    def update(frame):
        if not running: plt.close(fig); return line_p, line_m
        with data_lock:
            t_d, p_d, m_d, title = list(time_data), list(pressure_data), list(position_data), current_mode_text
        ax1.set_title(f"Status: {title}")
        if t_d:
            line_p.set_data(t_d, p_d); line_m.set_data(t_d, m_d)
            curr_t = t_d[-1]
            if curr_t > 10: ax1.set_xlim(curr_t - 10, curr_t)
            if p_d:
                c_min, c_max = min(p_d), max(p_d)
                amp = max(c_max - c_min, 0.2)
                ax1.set_ylim(c_min - amp*0.1, c_max + amp*0.1)
        return line_p, line_m

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.show()
    running = False; t.join(timeout=1.0)

if __name__ == "__main__":
    main()