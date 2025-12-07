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

# --- Parameters from Thesis ---
sampling_rate = 1.0 / 60.0  # Approx 0.0167 seconds (60Hz)
lowpass_fs = 60.0           
lowpass_cutoff = 2.0        
lowpass_order = 4           

# --- NEW: Detection Thresholds ---
# 1. Hysteresis (遲滯): 必須超過這個變化量才認定為狀態切換 (防止亂跳)
HYSTERESIS_THRESHOLD = 0.03  
# 2. Stability (穩定): 如果變化量小於此值，認定為閉氣 (防止馬達抖動)
STABILITY_THRESHOLD = 0.003  

# Other parameters
sampling_window = 4
increase_breath_time = 0.5
linear_actuator_max_distance = 50
success_threshold = 15
fail_threshold = 50

# --- Real-time Filter Class ---
class RealTimeFilter:
    def __init__(self, order, cutoff, fs):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.zi = lfilter_zi(self.b, self.a)
    
    def process(self, value):
        filtered_value, self.zi = lfilter(self.b, self.a, [value], zi=self.zi)
        return filtered_value[0]

# --- Helper Functions ---

def init_guide_phase(breath_times):
    if not breath_times:
        return 3.0 
    target_breath_time = np.median(breath_times)
    print(f"\n[系統] 進入引導階段 (Guide Phase)")
    print(f"[系統] 計算出的中位數呼吸時間: {target_breath_time:.2f} 秒")
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
        # Stop motor
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

# --- Modified Logic with Hysteresis & Breath Hold Detection ---
def advanced_mirror_logic(curr, prev, position, current_phase, last_peak, last_valley):
    """
    Returns: position, direction, new_phase, is_holding, updated_peak, updated_valley
    """
    direction = 0
    new_phase = current_phase
    is_holding = False
    
    # 1. Update Peak/Valley Records
    if curr > last_peak:
        last_peak = curr
    if curr < last_valley:
        last_valley = curr
        
    # 2. State Switching Logic (Schmitt Trigger)
    if current_phase == UserState.INHALE:
        # 只有當氣壓從最高點下降超過門檻，才切換到吐氣
        if curr < (last_peak - HYSTERESIS_THRESHOLD):
            new_phase = UserState.EXHALE
            last_valley = curr # 重置波谷記錄
            
    elif current_phase == UserState.EXHALE:
        # 只有當氣壓從最低點上升超過門檻，才切換到吸氣
        if curr > (last_valley + HYSTERESIS_THRESHOLD):
            new_phase = UserState.INHALE
            last_peak = curr # 重置波峰記錄

    # 3. Motor Control & Hold Detection
    # 判斷順時變化量 (Instant slope)
    delta = curr - prev
    
    if abs(delta) < STABILITY_THRESHOLD:
        # 變化太小 -> 閉氣/靜止 (Stop Motor)
        is_holding = True
        direction = 0 
    else:
        # 變化足夠 -> 依照目前的 Phase 移動
        is_holding = False
        if new_phase == UserState.INHALE:
            if position <= linear_actuator_max_distance:
                direction = 1
        else: # EXHALE
            if position >= 0:
                direction = -1

    move_linear_actuator(direction)
    position += direction
    
    return position, direction, new_phase, is_holding, last_peak, last_valley

def guide_breathing_logic(machine_breath, target_breath, position):
    direction = 0
    half_cycle = target_breath / 2.0
    
    if machine_breath < half_cycle: # Inhale phase
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

    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs)

    # State Variables
    machine_state = MachineState.MIRROR 
    print(">>> 目前階段: MIRROR (模仿階段 - 60秒)")
    print(">>> 包含閉氣偵測與防手震功能...")

    # Phase Tracking
    user_phase = UserState.EXHALE # 追蹤大方向 (Inhale phase vs Exhale phase)
    last_printed_status = ""      # 用來避免重複 print
    
    phase_start_time = time.time()
    
    # Peak/Valley Tracking for Hysteresis
    last_peak_pressure = -99999
    last_valley_pressure = 99999
    
    # Data Collection
    detected_breath_times = []
    current_breath_duration = 0
    
    # Actuator State
    la_position = 0
    la_direction = 0
    
    # Guide State
    target_breath_time = 3.0
    machine_breath_timer = 0
    
    # Prime filter
    first_read = bmp280.get_pressure()
    prev_filtered_pressure = rt_filter.process(first_read)
    
    # Initialize Peak/Valley with first read
    last_peak_pressure = prev_filtered_pressure
    last_valley_pressure = prev_filtered_pressure

    try:
        while True:
            loop_start = time.time()
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            
            # 1. Sensing & Filtering
            raw_pressure = bmp280.get_pressure()
            curr_filtered_pressure = rt_filter.process(raw_pressure)
            
            # 2. Logic & Machine State
            is_holding = False
            current_status_str = ""
            
            if machine_state == MachineState.MIRROR:
                # 使用新的 Advanced Logic
                la_position, la_direction, new_phase, is_holding, last_peak_pressure, last_valley_pressure = advanced_mirror_logic(
                    curr_filtered_pressure, 
                    prev_filtered_pressure, 
                    la_position, 
                    user_phase,
                    last_peak_pressure,
                    last_valley_pressure
                )
                
                # Check phase transition for duration calculation
                if user_phase == UserState.EXHALE and new_phase == UserState.INHALE:
                    # Cycle Start
                    if current_breath_duration > 0.5:
                        detected_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                
                user_phase = new_phase
                
                # Determine String for Print
                if is_holding:
                    current_status_str = "HOLD"
                elif user_phase == UserState.INHALE:
                    current_status_str = "INHALE"
                else:
                    current_status_str = "EXHALE"

                # Check for Phase Transition (Time limit)
                if time.time() - phase_start_time >= 60.0:
                    print("\n" + "="*40)
                    print(">>> 時間到: 切換至 GUIDE (引導) 階段")
                    print("="*40 + "\n")
                    target_breath_time = init_guide_phase(detected_breath_times)
                    machine_state = MachineState.GUIDE
                    detected_breath_times = [] 

            elif machine_state == MachineState.GUIDE:
                # Guide 模式邏輯不變
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # 這裡的 user compliance check 依然可以運作，因為 user_phase 在外面也可以算
                # 但為了簡化，Guide 模式下我們暫時不印出 User 狀態，專注於引導
                current_status_str = "GUIDING"
                
                # ... (Evaluation logic same as before) ...
                if len(detected_breath_times) >= sampling_window:
                    eval_state, new_target = validate_stable(detected_breath_times, target_breath_time)
                    if eval_state == EvalState.SUCCESS:
                        print(f"[評估] 成功跟隨! 調整目標為更慢: {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    elif eval_state == EvalState.FAIL:
                        print(f"[評估] 跟隨失敗. 調整回使用者速度: {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    else:
                        detected_breath_times.pop(0)

            # --- PRINT LOGIC ---
            if current_status_str != last_printed_status:
                if current_status_str == "INHALE":
                    print(f"[{timestamp}] [偵測] 吸氣 (Inhale) ▲")
                elif current_status_str == "EXHALE":
                    print(f"[{timestamp}] [偵測] 吐氣 (Exhale) ▼")
                elif current_status_str == "HOLD":
                    print(f"[{timestamp}] [偵測] 閉氣 / 靜止 (Hold) -")
                last_printed_status = current_status_str
            # -------------------

            # Update Timing & Vars
            current_breath_duration += sampling_rate
            prev_filtered_pressure = curr_filtered_pressure
            
            # Loop Timing
            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n程式結束 (Keyboard Interrupt)")
        GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main()