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
    GUIDE = 1 # 只剩 Guide 階段

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
warmup_duration = 5.0 # 暖機 5 秒

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
    # 純時間控制，強制引導
    direction = 0
    half_cycle = target_breath / 2.0
    
    if machine_breath < half_cycle: # 吸氣階段
        if position <= linear_actuator_max_distance:
            direction = 1
        else:
            direction = 0
        machine_breath += sampling_rate
    
    elif machine_breath >= half_cycle and machine_breath < target_breath: # 吐氣階段
        if position >= 0:
            direction = -1
        else:
            direction = 0
        machine_breath += sampling_rate

    if machine_breath >= target_breath:
        machine_breath = 0 # 重置週期

    move_linear_actuator(direction)
    position += direction

    return machine_breath, position

def main():
    print("程式啟動中...")
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

    # 1. 濾波器初始化
    first_read = bmp280.get_pressure()
    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)

    # State Variables
    machine_state = MachineState.WARMUP 
    print(f">>> 系統暖機中 ({warmup_duration}秒)...")

    user_state = UserState.EXHALE
    program_start_time = time.time()
    
    # Data Collection
    detected_breath_times = []
    current_breath_duration = 0
    skip_first_breath = True # 依然跳過第一筆暖機後的資料

    # Actuator State
    la_position = 0
    
    # Guide State
    # 因為沒有 Mirror 測量，我們先設定一個預設值，例如 3.0 秒 (一般人的放鬆呼吸)
    target_breath_time = 3.0 
    machine_breath_timer = 0
    
    prev_filtered_pressure = rt_filter.process(first_read)

    try:
        while True:
            loop_start = time.time()
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            
            # 1. Sensing
            raw_pressure = bmp280.get_pressure()
            curr_filtered_pressure = rt_filter.process(raw_pressure)
            
            # 2. Detect User Breath Phase (背景偵測，用於評估是否有跟上)
            user_action = None
            if curr_filtered_pressure > prev_filtered_pressure:
                user_action = UserState.INHALE
            elif curr_filtered_pressure < prev_filtered_pressure:
                user_action = UserState.EXHALE
            
            # --- WARMUP PHASE ---
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0) # 停止
                
                # 同步 user_state
                if user_action is not None:
                    user_state = user_action

                if time.time() - program_start_time >= warmup_duration:
                    print("\n" + "="*40)
                    print(">>> 暖機完成！直接進入 GUIDE (引導) 階段")
                    print(f">>> 初始目標呼吸時間: {target_breath_time} 秒")
                    print("="*40 + "\n")
                    
                    machine_state = MachineState.GUIDE
                    current_breath_duration = 0
                    skip_first_breath = True
                    detected_breath_times = []

            # --- GUIDE PHASE (Direct Entry) ---
            elif machine_state == MachineState.GUIDE:
                # A. 馬達邏輯：強制引導 (Pacing)
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # B. 使用者偵測邏輯：檢查 Compliance (背景執行)
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        if skip_first_breath:
                            print(f"(忽略初始不完整呼吸: {current_breath_duration:.2f}s)")
                            skip_first_breath = False
                        else:
                            detected_breath_times.append(current_breath_duration)
                            # print(f"偵測到使用者呼吸: {current_breath_duration:.2f}s") # Optional Debug
                    
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

                # C. 評估邏輯：是否跟上？
                if len(detected_breath_times) >= sampling_window:
                    eval_state, new_target = validate_stable(detected_breath_times, target_breath_time)
                    
                    if eval_state == EvalState.SUCCESS:
                        print(f"[{timestamp}] 評估: 成功跟隨! 放慢引導速度至: {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    elif eval_state == EvalState.FAIL:
                        print(f"[{timestamp}] 評估: 跟隨失敗/不穩. 調整回使用者速度: {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    else:
                        detected_breath_times.pop(0)

            prev_filtered_pressure = curr_filtered_pressure
            
            # Timing
            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n程式結束")
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main()