#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import numpy as np
from enum import Enum
from scipy.signal import butter, lfilter, lfilter_zi

# --- GPIO & Sensor Imports ---
try:
    import RPi.GPIO as GPIO
    from bmp280 import BMP280
    from smbus2 import SMBus
except ImportError:
    # 為了方便在非 Pi 環境測試而不報錯
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
    current_action = "" # 用來記錄當下的動作字串
    
    if timer < half:
        # 吸氣階段：馬達伸出
        if pos <= linear_actuator_max_distance: direct = 1
        else: direct = 0
        timer += sampling_rate
        current_action = "INHALE"
    elif timer >= half and timer < target:
        # 吐氣階段：馬達縮回
        if pos >= 0: direct = -1
        else: direct = 0
        timer += sampling_rate
        current_action = "EXHALE"
    
    if timer >= target: timer = 0

    move_linear_actuator(direct)
    pos += direct
    
    # 回傳當下的動作狀態
    return timer, pos, current_action

# --- Main Logic ---
# 修改：增加 stop_event (用於外部停止) 和 msg_callback (用於回傳訊號)
def main(stop_event=None, msg_callback=None):
    print(">>> 呼吸控制系統啟動...")
    
    # GPIO 初始化
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(en, GPIO.OUT)
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        
        p = GPIO.PWM(en, 800)
        p.start(100)
    except Exception as e:
        print(f"GPIO Init Error: {e}")
        return
    
    # Sensor 初始化
    try:
        bus = SMBus(1)
        bmp280 = BMP280(i2c_dev=bus)
        bmp280.setup(mode="forced")
        first_read = bmp280.get_pressure()
        print(">>> 感測器連接成功")
    except Exception as e:
        print(f"!!! Sensor Error: {e}")
        # 如果感測器壞了也要清理 GPIO
        p.stop()
        GPIO.cleanup()
        return

    # 變數初始化
    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)
    machine_state = MachineState.WARMUP
    user_state = UserState.EXHALE
    
    program_start_time = time.time()
    mirror_start_time = 0
    
    mirror_breath_times = []    
    detected_breath_times = []  
    current_breath_duration = 0
    skip_first_breath = True

    la_position = 0
    target_breath_time = 3.0 
    machine_breath_timer = 0
    
    prev_filtered = rt_filter.process(first_read)
    
    # 用來記錄上次發送給 Unity 的狀態，避免重複發送
    last_sent_action = ""

    print(f">>> 系統暖機中 ({warmup_duration}秒)...")

    try:
        # 迴圈條件：如果 stop_event 被設定了，就跳出迴圈
        while not (stop_event and stop_event.is_set()):
            loop_start = time.time()
            
            # 讀取與濾波
            try:
                raw = bmp280.get_pressure()
                curr_filtered = rt_filter.process(raw)
            except OSError:
                print("Sensor Read Error")
                continue
            
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
                    print(">>> [系統] 暖機完成 -> 進入 MIRROR 模式")
                    machine_state = MachineState.MIRROR
                    mirror_start_time = time.time()
                    current_breath_duration = 0

            elif machine_state == MachineState.MIRROR:
                move_linear_actuator(0) 
                
                if user_state == UserState.EXHALE and user_action == UserState.INHALE:
                    if current_breath_duration > 0.8: 
                        mirror_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and user_action == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

                if time.time() - mirror_start_time >= mirror_duration:
                    if len(mirror_breath_times) > 0:
                        target_breath_time = np.mean(mirror_breath_times)
                        print(f">>> [結果] Mirror 結束. 平均頻率: {target_breath_time:.2f} 秒")
                    else:
                        target_breath_time = 4.0
                        print(f">>> [結果] 使用預設值: 4.00 秒")
                    
                    machine_state = MachineState.GUIDE
                    print(f">>> [系統] 進入 GUIDE 模式")
                    current_breath_duration = 0
                    skip_first_breath = True

            elif machine_state == MachineState.GUIDE:
                # 馬達開始引導，並取得當前動作 (INHALE/EXHALE)
                machine_breath_timer, la_position, action = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # --- [新增功能] 發送訊號給 Unity ---
                if msg_callback and action != last_sent_action:
                    msg_callback(f"ANIM:{action}\n")
                    last_sent_action = action
                # ----------------------------------
                
                # 記錄使用者表現
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

                if len(detected_breath_times) >= sampling_window:
                    eval_st, new_target = validate_stable(detected_breath_times, target_breath_time)
                    if eval_st == EvalState.SUCCESS:
                        print(f">>> [調整] 穩定! 加速: {target_breath_time:.2f} -> {new_target:.2f}")
                        target_breath_time = new_target
                        detected_breath_times = []
                    elif eval_st == EvalState.FAIL:
                        print(f">>> [調整] 混亂! 減速: {target_breath_time:.2f} -> {new_target:.2f}")
                        target_breath_time = new_target
                        detected_breath_times = []
                    else:
                        detected_breath_times.pop(0)

            prev_filtered = curr_filtered

            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n>>> 使用者中斷 (Ctrl+C)")
    except Exception as e:
        print(f"\n!!! Runtime Error: {e}")
    finally:
        print(">>> 清理 GPIO...")
        p.stop()
        GPIO.cleanup()
        print(">>> 呼吸程式結束")

if __name__ == "__main__":
    main()