#!/usr/bin/env python

import sys
import time
from enum import Enum
import RPi.GPIO as GPIO
from bmp280 import BMP280
import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi
import threading
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

class MachineState(Enum):
    WARMUP = -1 # 新增暖機階段
    MIRROR = 0
    GUIDE = 1

class UserState(Enum):
    INHALE = 0
    EXHALE = 1

class EvalState(Enum):
    NONE = 0
    FAIL = 1
    SUCCESS = 2

# --- Global Shared Data for Plotting ---
MAX_POINTS = 600 # 10 seconds @ 60Hz
pressure_data = collections.deque(maxlen=MAX_POINTS)
position_data = collections.deque(maxlen=MAX_POINTS)
time_data = collections.deque(maxlen=MAX_POINTS)
running = True # Global control flag

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
warmup_duration = 5.0 # 暖機 5 秒，確保濾波器穩定

# --- Real-time Filter Class (修正初始化問題) ---
class RealTimeFilter:
    def __init__(self, order, cutoff, fs, initial_value=0.0):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        # [關鍵修正] 乘上初始值，讓濾波器直接從目前的氣壓開始，消除前幾秒的巨大誤差
        self.zi = lfilter_zi(self.b, self.a) * initial_value
    
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
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

def mirror_breathing_logic(curr_filtered, prev_filtered, position, direction):
    # 回歸最單純的斜率判斷 (拿掉閉氣偵測)
    is_inhaling = curr_filtered > prev_filtered
    is_exhaling = curr_filtered < prev_filtered

    if is_inhaling: # Inhale
        if position <= linear_actuator_max_distance:
            direction = 1
        else:
            direction = 0
    elif is_exhaling: # Exhale
        if position >= 0:
            direction = -1
        else:
            direction = 0

    move_linear_actuator(direction)
    position += direction
    return position, direction

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

def control_loop():
    global running
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

    # 1. 取得初始值，讓濾波器正確初始化 (這是數據準確的關鍵)
    first_read = bmp280.get_pressure()
    rt_filter = RealTimeFilter(lowpass_order, lowpass_cutoff, lowpass_fs, initial_value=first_read)

    # State Variables
    machine_state = MachineState.WARMUP 
    print(f">>> 系統暖機中 ({warmup_duration}秒)... 請保持自然呼吸")

    user_state = UserState.EXHALE
    last_printed_user_state = None 
    
    phase_start_time = 0 # 之後會重設
    program_start_time = time.time()
    
    # Data Collection
    detected_breath_times = []
    current_breath_duration = 0
    
    # [新變數] 用來確保第一筆記錄是完整的
    skip_first_breath = True 

    # Actuator State
    la_position = 0
    la_direction = 0
    
    # Guide State
    target_breath_time = 3.0
    machine_breath_timer = 0
    
    prev_filtered_pressure = rt_filter.process(first_read)

    prev_filtered_pressure = rt_filter.process(first_read)

    try:
        while running:
            loop_start = time.time()
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            
            # 1. Sensing & Filtering
            raw_pressure = bmp280.get_pressure()
            curr_filtered_pressure = rt_filter.process(raw_pressure)
            
            # 2. Logic & States
            current_logic_state = last_printed_user_state # Default

            # --- WARMUP PHASE ---
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0) # 暖機時不動
                
                # 簡單的狀態追蹤，為了讓切換時 user_state 是正確的
                if curr_filtered_pressure > prev_filtered_pressure:
                    user_state = UserState.INHALE
                else:
                    user_state = UserState.EXHALE

                if time.time() - program_start_time >= warmup_duration:
                    print("\n" + "="*40)
                    print(">>> 暖機完成! 進入 MIRROR (模仿階段 - 60秒)")
                    print(">>> 開始記錄數據 (已自動忽略第一筆不完整呼吸)")
                    print("="*40 + "\n")
                    
                    machine_state = MachineState.MIRROR
                    phase_start_time = time.time()
                    current_breath_duration = 0 
                    skip_first_breath = True # 再次確認要跳過第一筆

            # --- MIRROR PHASE ---
            elif machine_state == MachineState.MIRROR:
                # 判斷吸吐氣
                if curr_filtered_pressure > prev_filtered_pressure:
                    current_logic_state = UserState.INHALE
                elif curr_filtered_pressure < prev_filtered_pressure:
                    current_logic_state = UserState.EXHALE
                else:
                    current_logic_state = last_printed_user_state

                # Print 狀態改變
                if current_logic_state != last_printed_user_state:
                    if current_logic_state == UserState.INHALE:
                        print(f"[{timestamp}] [偵測] 吸氣 (Inhale) ▲")
                    elif current_logic_state == UserState.EXHALE:
                        print(f"[{timestamp}] [偵測] 吐氣 (Exhale) ▼")
                    last_printed_user_state = current_logic_state
                
                # 計算呼吸時間 (Algorithm 3)
                if user_state == UserState.EXHALE and current_logic_state == UserState.INHALE:
                    # 週期結束 (吐氣轉吸氣瞬間)
                    if current_breath_duration > 0.5: 
                        if skip_first_breath:
                            # 忽略第一筆，因為它包含了暖機切換過來的「半口氣」
                            print(f"[{timestamp}] (忽略初始不完整呼吸: {current_breath_duration:.2f}s)")
                            skip_first_breath = False
                        else:
                            # 這才是真正的完整呼吸
                            detected_breath_times.append(current_breath_duration)
                            print(f"[{timestamp}] >> 記錄有效呼吸: {current_breath_duration:.2f}s")
                    
                    current_breath_duration = 0
                    user_state = UserState.INHALE

                elif user_state == UserState.INHALE and current_logic_state == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

                # 馬達控制
                la_position, la_direction = mirror_breathing_logic(
                    curr_filtered_pressure, prev_filtered_pressure, 
                    la_position, la_direction
                )
                
                # 超時切換
                if time.time() - phase_start_time >= 60.0:
                    print("\n" + "="*40)
                    print(">>> 時間到: 切換至 GUIDE (引導) 階段")
                    print("="*40 + "\n")
                    target_breath_time = init_guide_phase(detected_breath_times)
                    machine_state = MachineState.GUIDE
                    detected_breath_times = [] 

            # --- GUIDE PHASE ---
            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # Guide 階段依然在背景計算 User Compliance
                # 這裡使用簡化的邏輯 (不需印出狀態，只要收集數據)
                logic_phase_guide = user_state
                if curr_filtered_pressure > prev_filtered_pressure:
                    logic_phase_guide = UserState.INHALE
                elif curr_filtered_pressure < prev_filtered_pressure:
                    logic_phase_guide = UserState.EXHALE
                
                if user_state == UserState.EXHALE and logic_phase_guide == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        detected_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and logic_phase_guide == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

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

            prev_filtered_pressure = curr_filtered_pressure

            # --- Data for Plotting ---
            pressure_data.append(curr_filtered_pressure)
            position_data.append(la_position)
            # Use relative time for easier plotting
            time_data.append(time.time() - program_start_time)
            
            # Timing
            elapsed = time.time() - loop_start
            sleep_time = sampling_rate - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n控制線程結束 (Keyboard Interrupt)")
    except Exception as e:
        print(f"\n發生錯誤: {e}")
    finally:
        print("清理 GPIO...")
        GPIO.cleanup()
        running = False

def main():
    global running
    print("程式啟動中... (主視窗顯示波形，背景執行控制)")

    # Start control thread
    t = threading.Thread(target=control_loop)
    t.daemon = True
    t.start()

    # Setup Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    ax1.set_title("Real-time Pressure & Position")
    ax1.set_ylabel("Pressure (hPa)")
    line_pressure, = ax1.plot([], [], 'b-', label='Pressure')
    ax1.legend(loc='upper right')
    
    ax2.set_ylabel("Admin Motor Position")
    ax2.set_xlabel("Time (s)")
    line_position, = ax2.plot([], [], 'r-', label='Position')
    ax2.legend(loc='upper right')

    def init():
        ax1.set_xlim(0, 10)
        # Assuming pressure range, can auto-scale or set fixed
        # ax1.set_ylim(950, 1050) 
        ax2.set_ylim(-5, 60) # Linear actuator max is 50
        return line_pressure, line_position

    def update(frame):
        if not running:
            plt.close(fig)
            return line_pressure, line_position

        # Copy data to avoid race condition (simple copy is usually enough)
        t = list(time_data)
        p = list(pressure_data)
        pos = list(position_data)

        if t:
            line_pressure.set_data(t, p)
            line_position.set_data(t, pos)
            
            # Auto-scroll x-axis
            current_time = t[-1]
            if current_time > 10:
                ax1.set_xlim(current_time - 10, current_time)
            else:
                ax1.set_xlim(0, 10)
            
            # Auto-scale Y for pressure if needed
            if p:
                min_p, max_p = min(p), max(p)
                margin = (max_p - min_p) * 0.1
                if margin == 0: margin = 1
                ax1.set_ylim(min_p - margin, max_p + margin)

        return line_pressure, line_position

    ani = animation.FuncAnimation(fig, update, init_func=init, blit=False, interval=50) # 20fps
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    
    print("主視窗關閉，停止程式...")
    running = False
    t.join()

if __name__ == "__main__":
    main()