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
    MIRROR = 0  # 新增鏡像階段
    GUIDE = 1 

class UserState(Enum):
    INHALE = 0
    EXHALE = 1

class EvalState(Enum):
    NONE = 0
    FAIL = 1
    SUCCESS = 2

# --- Global Shared Data & Lock ---
MAX_POINTS = 600
data_lock = threading.Lock()

pressure_data = collections.deque(maxlen=MAX_POINTS)
position_data = collections.deque(maxlen=MAX_POINTS)
time_data = collections.deque(maxlen=MAX_POINTS)
current_mode_text = "Initializing" # 用於圖表顯示當前狀態
running = True 

# --- Pin Definition ---
in1 = 23
in2 = 24
en = 25

# --- Parameters ---
sampling_rate = 1.0 / 60.0  
lowpass_fs = 60.0           
lowpass_cutoff = 2.0        
lowpass_order = 4           

sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50

warmup_duration = 5.0
mirror_duration = 60.0 # 鏡像偵測時間

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
    
    next_target = target_breath_time
    state = EvalState.NONE

    if np.all(np.abs(deviations) <= success_threshold):
        state = EvalState.SUCCESS
        next_target = target_breath_time + increase_breath_time
    elif np.any(np.abs(deviations) > fail_threshold):
        state = EvalState.FAIL
        next_target = np.mean(recent) 
        
    return state, next_target

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
    except Exception:
        pass

def guide_breathing_logic(timer, target, pos):
    direct = 0
    half = target / 2.0
    
    if timer < half:
        if pos <= linear_actuator_max_distance: direct = 1
        else: direct = 0
        timer += sampling_rate
    elif timer >= half and timer < target:
        if pos >= 0: direct = -1
        else: direct = 0
        timer += sampling_rate
    
    if timer >= target: timer = 0

    move_linear_actuator(direct)
    pos += direct
    return timer, pos

def control_loop():
    global running, current_mode_text
    print(">>> 背景控制執行緒啟動...")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(en, GPIO.OUT)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    
    p = GPIO.PWM(en, 800)
    p.start(100)
    
    try:
        bus = SMBus(1)
        bmp280 = BMP280(i2c_dev=bus)
        bmp280.setup(mode="forced")
        first_read = bmp280.get_pressure()
    except Exception as e:
        print(f"Sensor Error: {e}")
        running = False
        return

    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)

    machine_state = MachineState.WARMUP
    current_mode_text = "WARMUP"
    user_state = UserState.EXHALE
    
    program_start_time = time.time()
    mirror_start_time = 0
    
    mirror_breath_times = []    # Mirror 階段記錄用
    detected_breath_times = []  # Guide 階段評估用
    current_breath_duration = 0
    skip_first_breath = True

    la_position = 0
    target_breath_time = 3.0 # 預設值，會被 Mirror 結果覆蓋
    machine_breath_timer = 0
    
    prev_filtered = rt_filter.process(first_read)

    print(f">>> 系統暖機中 ({warmup_duration}秒)...")

    try:
        while running:
            loop_start = time.time()
            ts = time.strftime("%H:%M:%S", time.localtime())
            
            raw = bmp280.get_pressure()
            curr_filtered = rt_filter.process(raw)
            
            # 判斷使用者吸吐動作
            user_action = None
            if curr_filtered > prev_filtered:
                user_action = UserState.INHALE
            elif curr_filtered < prev_filtered:
                user_action = UserState.EXHALE

            # --- 狀態機邏輯 ---
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0)
                if user_action is not None:
                    user_state = user_action
                if time.time() - program_start_time >= warmup_duration:
                    print(f"[{ts}] 暖機完成 -> 進入 MIRROR 偵測 (60秒)")
                    machine_state = MachineState.MIRROR
                    current_mode_text = "MIRRORING (Keep Normal Breath)"
                    mirror_start_time = time.time()
                    current_breath_duration = 0

            elif machine_state == MachineState.MIRROR:
                move_linear_actuator(0) # Mirror 階段馬達不動
                
                # 偵測呼吸並計時
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.8: # 略微過濾雜訊
                        mirror_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

                # 判斷 Mirror 60秒是否結束
                if time.time() - mirror_start_time >= mirror_duration:
                    if len(mirror_breath_times) > 0:
                        target_breath_time = np.mean(mirror_breath_times)
                        print(f"[{ts}] Mirror 結束. 平均頻率: {target_breath_time:.2f}s")
                    else:
                        target_breath_time = 4.0 # 沒抓到則用保險值
                        print(f"[{ts}] 未抓到有效呼吸，使用預設 4.0s")
                    
                    machine_state = MachineState.GUIDE
                    current_mode_text = f"GUIDING (Target: {target_breath_time:.1f}s)"
                    current_breath_duration = 0
                    skip_first_breath = True

            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # 記錄使用者在 Guide 階段的表現
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        if skip_first_breath:
                            skip_first_breath = False
                        else:
                            detected_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

                # 每累積一定次數呼吸，評估是否增加難度
                if len(detected_breath_times) >= sampling_window:
                    eval_st, new_target = validate_stable(detected_breath_times, target_breath_time)
                    if eval_st == EvalState.SUCCESS:
                        print(f"[{ts}] 穩定! 挑戰更慢: {new_target:.2f}s")
                        target_breath_time = new_target
                        current_mode_text = f"GUIDING (Target: {target_breath_time:.1f}s)"
                        detected_breath_times = []
                    elif eval_st == EvalState.FAIL:
                        print(f"[{ts}] 不穩定. 調整回: {new_target:.2f}s")
                        target_breath_time = new_target
                        current_mode_text = f"GUIDING (Target: {target_breath_time:.1f}s)"
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
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"\nControl Loop Error: {e}")
    finally:
        print("清理 GPIO...")
        try: p.stop()
        except: pass
        try: GPIO.cleanup()
        except: pass
        running = False

def main():
    global running
    print("程式啟動中... (SSH X11 Mode)")

    t = threading.Thread(target=control_loop)
    t.daemon = True
    t.start()

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    ax1.get_yaxis().get_major_formatter().set_useOffset(False)
    line_p, = ax1.plot([], [], 'b-', lw=2)
    line_m, = ax2.plot([], [], 'r-', lw=2)
    
    ax1.set_xlim(0, 10)
    ax2.set_ylim(-5, 60)
    ax2.set_ylabel("Motor Pos")
    ax2.set_xlabel("Time (s)")

    def update(frame):
        if not running:
            plt.close(fig)
            return line_p, line_m

        with data_lock:
            t_data = list(time_data)
            p_data = list(pressure_data)
            m_data = list(position_data)
            current_title = current_mode_text

        ax1.set_title(f"Breathing Monitor - Status: {current_title}")

        if t_data:
            line_p.set_data(t_data, p_data)
            line_m.set_data(t_data, m_data)
            
            curr_t = t_data[-1]
            if curr_t > 10:
                ax1.set_xlim(curr_t - 10, curr_t)
            
            if p_data:
                curr_min, curr_max = min(p_data), max(p_data)
                amplitude = curr_max - curr_min
                min_range = 0.2
                if amplitude < min_range:
                    center = (curr_max + curr_min) / 2.0
                    ax1.set_ylim(center - min_range/2, center + min_range/2)
                else:
                    padding = amplitude * 0.1
                    ax1.set_ylim(curr_min - padding, curr_max + padding)

        return line_p, line_m

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    running = False
    t.join(timeout=1.0)

if __name__ == "__main__":
    main()