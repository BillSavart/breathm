#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import threading
import collections
from enum import Enum
import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi

# --- Matplotlib 設定 (必須在 import pyplot 之前) ---
import matplotlib
# 強制使用 TkAgg 後端，這對 X11 Forwarding (SSH -X) 相容性最好
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- GPIO & Sensor Imports ---
try:
    import RPi.GPIO as GPIO
    from bmp280 import BMP280
    from smbus2 import SMBus
except ImportError:
    # 為了讓你在沒裝硬體的電腦也能測試 run code，這邊做個簡單的 fallback 提示
    from smbus import SMBus # 舊版相容
    print("Warning: SMBus or BMP280 library mismatch, ensure environment is set.")

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

# --- Global Shared Data & Lock ---
MAX_POINTS = 600
# 加入 Lock 防止讀寫衝突 (Thread Safety)
data_lock = threading.Lock()

pressure_data = collections.deque(maxlen=MAX_POINTS)
position_data = collections.deque(maxlen=MAX_POINTS)
time_data = collections.deque(maxlen=MAX_POINTS)
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
def init_guide_phase(breath_times):
    if not breath_times:
        return 3.0 
    target = np.median(breath_times)
    print(f"\n[系統] 進入引導階段 (Guide Phase)")
    print(f"[系統] 計算中位數呼吸: {target:.2f} 秒")
    return target

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
    # 簡單保護，避免 GPIO 已經被清空後呼叫
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

def mirror_breathing_logic(curr, prev, pos, direct):
    is_inhale = curr > prev
    is_exhale = curr < prev
    
    if is_inhale:
        if pos <= linear_actuator_max_distance: direct = 1
        else: direct = 0
    elif is_exhale:
        if pos >= 0: direct = -1
        else: direct = 0
        
    move_linear_actuator(direct)
    pos += direct
    return pos, direct

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
    global running
    print(">>> 背景控制執行緒啟動...")
    
    # GPIO Setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(en, GPIO.OUT)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    
    p = GPIO.PWM(en, 800)
    p.start(100)
    
    # Sensor Setup
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

    # Variables
    machine_state = MachineState.WARMUP
    user_state = UserState.EXHALE
    last_printed_user_state = None
    
    phase_start_time = 0
    program_start_time = time.time()
    
    detected_breath_times = []
    current_breath_duration = 0
    skip_first_breath = True

    la_position = 0
    la_direction = 0
    
    target_breath_time = 3.0
    machine_breath_timer = 0
    
    prev_filtered = rt_filter.process(first_read)

    print(f">>> 系統暖機中 ({warmup_duration}秒)...")

    try:
        while running:
            loop_start = time.time()
            ts = time.strftime("%H:%M:%S", time.localtime())
            
            # 1. Sensing
            raw = bmp280.get_pressure()
            curr_filtered = rt_filter.process(raw)
            
            # 2. Logic
            logic_state = last_printed_user_state

            # --- WARMUP ---
            if machine_state == MachineState.WARMUP:
                move_linear_actuator(0)
                if curr_filtered > prev_filtered: user_state = UserState.INHALE
                else: user_state = UserState.EXHALE

                if time.time() - program_start_time >= warmup_duration:
                    print(f"\n[{ts}] 暖機完成 -> MIRROR 模式")
                    machine_state = MachineState.MIRROR
                    phase_start_time = time.time()
                    current_breath_duration = 0
                    skip_first_breath = True

            # --- MIRROR ---
            elif machine_state == MachineState.MIRROR:
                if curr_filtered > prev_filtered: logic_state = UserState.INHALE
                elif curr_filtered < prev_filtered: logic_state = UserState.EXHALE
                
                if logic_state != last_printed_user_state:
                    sym = "▲" if logic_state == UserState.INHALE else "▼"
                    print(f"[{ts}] {logic_state.name} {sym}")
                    last_printed_user_state = logic_state

                # Breath detection logic
                if user_state == UserState.EXHALE and logic_state == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        if skip_first_breath:
                            skip_first_breath = False
                        else:
                            detected_breath_times.append(current_breath_duration)
                            print(f"   >> 有效呼吸: {current_breath_duration:.2f}s")
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and logic_state == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate
                
                la_position, la_direction = mirror_breathing_logic(
                    curr_filtered, prev_filtered, la_position, la_direction
                )

                if time.time() - phase_start_time >= 60.0:
                    print(f"\n[{ts}] MIRROR 結束 -> GUIDE 模式")
                    target_breath_time = init_guide_phase(detected_breath_times)
                    machine_state = MachineState.GUIDE
                    detected_breath_times = []

            # --- GUIDE ---
            elif machine_state == MachineState.GUIDE:
                machine_breath_timer, la_position = guide_breathing_logic(
                    machine_breath_timer, target_breath_time, la_position
                )
                
                # Background compliance check (simplified)
                guide_logic = user_state
                if curr_filtered > prev_filtered: guide_logic = UserState.INHALE
                elif curr_filtered < prev_filtered: guide_logic = UserState.EXHALE
                
                if user_state == UserState.EXHALE and guide_logic == UserState.INHALE:
                    if current_breath_duration > 0.5:
                        detected_breath_times.append(current_breath_duration)
                    current_breath_duration = 0
                    user_state = UserState.INHALE
                elif user_state == UserState.INHALE and guide_logic == UserState.EXHALE:
                    user_state = UserState.EXHALE
                
                current_breath_duration += sampling_rate

                if len(detected_breath_times) >= sampling_window:
                    eval_st, new_target = validate_stable(detected_breath_times, target_breath_time)
                    if eval_st != EvalState.NONE:
                        print(f"[{ts}] 評估結果: {eval_st.name}, 新目標: {new_target:.2f}s")
                        target_breath_time = new_target
                        detected_breath_times = []
                    else:
                        detected_breath_times.pop(0)

            prev_filtered = curr_filtered

            # --- Store Data (Thread Safe) ---
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
        print("清理 GPIO 資源...")
        # [關鍵修正] 安全關閉 PWM 與 GPIO
        try:
            p.stop()
        except:
            pass
        try:
            GPIO.cleanup()
        except:
            pass
        running = False

def main():
    global running
    print("程式啟動中... (Mac XQuartz Mode)")

    # 啟動控制執行緒
    t = threading.Thread(target=control_loop)
    t.daemon = True
    t.start()

    # 繪圖設定
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    # [優化 1] 設定標題與 Y 軸格式，取消科學記號顯示 (避免看到 +1.013e3 這種東西)
    ax1.set_title("Real-time Breathing Pressure")
    ax1.set_ylabel("Pressure (hPa)")
    ax1.get_yaxis().get_major_formatter().set_useOffset(False) # 禁用偏移顯示
    
    line_p, = ax1.plot([], [], 'b-', lw=2) # lw=2 加粗線條讓波形更明顯
    
    ax2.set_ylabel("Motor Pos")
    ax2.set_xlabel("Time (s)")
    line_m, = ax2.plot([], [], 'r-', lw=2)
    
    # 初始設定
    ax1.set_xlim(0, 10)
    # 這裡依照你的解釋，保留 Motor 的視覺留白
    ax2.set_ylim(-5, 60) 

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
            
            # X軸捲動邏輯 (保持原本)
            curr_t = t_data[-1]
            if curr_t > 10:
                ax1.set_xlim(curr_t - 10, curr_t)
            
            # [優化 2] 超級 Adaptive 的 Y 軸縮放邏輯
            if p_data:
                # 只看「目前視窗內」的數據來決定縮放 (取最後 10 秒左右的數據點)
                # 假設 sampling_rate 約 60Hz，10秒約 600點 (也就是 deque 的長度)
                curr_min = min(p_data)
                curr_max = max(p_data)
                
                # 計算波動範圍
                amplitude = curr_max - curr_min
                
                # 設定一個「最小顯示範圍」防止放大雜訊
                # 如果波動小於 0.2 hPa，我們就強制顯示 0.2 的範圍，避免線條亂抖
                min_display_range = 0.2 
                
                if amplitude < min_display_range:
                    center = (curr_max + curr_min) / 2.0
                    display_min = center - (min_display_range / 2.0)
                    display_max = center + (min_display_range / 2.0)
                else:
                    # 如果波動夠大，就緊貼數據，只留上下 5% 的空間
                    padding = amplitude * 0.05
                    display_min = curr_min - padding
                    display_max = curr_max + padding
                
                ax1.set_ylim(display_min, display_max)

        return line_p, line_m

    ani = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False
    )
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    
    print("主視窗關閉，程式結束。")
    running = False
    t.join(timeout=1.0)

if __name__ == "__main__":
    main()