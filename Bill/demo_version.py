#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import threading
import collections
import numpy as np
from enum import Enum
from scipy.signal import butter, lfilter, lfilter_zi

# --- Matplotlib (支援 X11) ---
import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- GPIO & Sensor ---
try:
    import RPi.GPIO as GPIO
    from bmp280 import BMP280
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus 
    print("Warning: Library mismatch (Testing mode).")

# --- 變數與常數 ---
class UserState(Enum):
    INHALE = 0
    EXHALE = 1

class EvalState(Enum):
    NONE = 0
    FAIL = 1
    SUCCESS = 2

MAX_POINTS = 600
data_lock = threading.Lock()
pressure_data = collections.deque(maxlen=MAX_POINTS)
position_data = collections.deque(maxlen=MAX_POINTS)
time_data = collections.deque(maxlen=MAX_POINTS)
running = True 

in1, in2, en = 23, 24, 25
sampling_rate = 1.0 / 60.0  
lowpass_fs = 60.0           
lowpass_cutoff = 2.0        
lowpass_order = 4           
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50

# --- Filter ---
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
    if len(breath_times) < sampling_window: return EvalState.NONE, target_breath_time
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

# --- 核心邏輯 Loop ---
def control_loop(queue):
    global running
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
        running = False
        return

    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)
    user_state = UserState.EXHALE
    
    program_start_time = time.time()
    detected_breath_times = []
    current_breath_duration = 0
    skip_first_breath = True
    la_position = 0
    target_breath_time = 3.0
    machine_breath_timer = 0
    prev_filtered = rt_filter.process(first_read)
    last_sent_action = ""

    print(">>> Demo Version Logic Started")

    try:
        while running:
            loop_start = time.time()
            raw = bmp280.get_pressure()
            curr_filtered = rt_filter.process(raw)
            
            user_action = None
            if curr_filtered > prev_filtered: user_action = UserState.INHALE
            elif curr_filtered < prev_filtered: user_action = UserState.EXHALE

            machine_breath_timer, la_position, action = guide_breathing_logic(
                machine_breath_timer, target_breath_time, la_position
            )
            
            if action != last_sent_action:
                queue.put(f"ANIM:{action}\n")
                last_sent_action = action

            if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                if current_breath_duration > 0.5:
                    if skip_first_breath: skip_first_breath = False
                    else: detected_breath_times.append(current_breath_duration)
                current_breath_duration = 0
                user_state = UserState.INHALE
            elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                user_state = UserState.EXHALE
            
            current_breath_duration += sampling_rate

            if len(detected_breath_times) >= sampling_window:
                eval_st, new_target = validate_stable(detected_breath_times, target_breath_time)
                if eval_st == EvalState.SUCCESS:
                    target_breath_time = new_target
                    detected_breath_times = []
                elif eval_st == EvalState.FAIL:
                    target_breath_time = new_target
                    detected_breath_times = []
                else:
                    detected_breath_times.pop(0)

            prev_filtered = curr_filtered
            with data_lock:
                pressure_data.append(curr_filtered)
                position_data.append(la_position)
                time_data.append(time.time() - program_start_time)

            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0: time.sleep(sleep_time)
            
    finally:
        try: p.stop()
        except: pass
        try: GPIO.cleanup()
        except: pass

# --- 新增的 GUI 接口 ---
def main_gui(queue):
    global running
    running = True
    
    t = threading.Thread(target=control_loop, args=(queue,), daemon=True)
    t.start()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    ax1.set_title("Demo Version (Guide Mode)")
    ax1.get_yaxis().get_major_formatter().set_useOffset(False)
    
    line_p, = ax1.plot([], [], 'b-', lw=2)
    line_m, = ax2.plot([], [], 'r-', lw=2)
    
    ax1.set_xlim(0, 10)
    ax2.set_ylim(-5, 60)
    ax2.set_ylabel("Pos")

    def update(frame):
        if not running:
            plt.close(fig)
            return line_p, line_m
        with data_lock:
            t_data = list(time_data)
            p_data = list(pressure_data)
            m_data = list(position_data)
        if t_data:
            line_p.set_data(t_data, p_data)
            line_m.set_data(t_data, m_data)
            curr_t = t_data[-1]
            if curr_t > 10: ax1.set_xlim(curr_t - 10, curr_t)
            if p_data:
                curr_min, curr_max = min(p_data), max(p_data)
                mid = (curr_max + curr_min)/2
                ax1.set_ylim(mid - 0.2, mid + 0.2)
        return line_p, line_m

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    try: plt.show()
    except KeyboardInterrupt: pass
    running = False
    t.join(timeout=1.0)
    print("Demo Process Ended")