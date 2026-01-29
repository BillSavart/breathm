#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import multiprocessing
import time
import os
import signal

# --- 匯入各個模式的模組 ---
# 請確保 fix_version.py, demo_version.py, demo_with_mirror.py 都在同目錄
try:
    import fix_version
    import demo_version
    import demo_with_mirror
except ImportError as e:
    print(f"Import Error: {e}")
    print("請確認 fix_version.py, demo_version.py, demo_with_mirror.py 都在同一資料夾")

# 設定 IP 與 Port
HOST = "0.0.0.0" 
PORT = 5005

# 全域變數管理
current_process = None      # 用於 GUI 模式 (Multiprocessing)
current_thread = None       # 用於無 GUI 模式 (Threading)
stop_event = threading.Event() # 用於控制 Thread 停止
msg_queue = None            # 用於接收子進程傳來的訊號
active_connection = None    # 當前的 Unity 連線

# --- 訊息轉發橋樑 (Bridge) ---
def message_bridge_loop(queue):
    """
    這是一個背景執行緒，專門監聽子進程(圖表)傳來的 Queue。
    只要收到 "ANIM:..." 訊號，就立刻透過 Socket 轉發給 Unity。
    """
    global active_connection
    print("[SERVER] Message bridge started.")
    while True:
        try:
            # 讀取訊息 (會在此等待直到有訊息)
            msg = queue.get()
            
            # 如果收到結束訊號，跳出迴圈
            if msg == "STOP_BRIDGE":
                break
                
            # 轉傳給 Unity
            if active_connection:
                try:
                    active_connection.sendall(msg.encode("utf-8"))
                except Exception as e:
                    print(f"[SERVER] Bridge Send Error: {e}")
                    
        except Exception as e:
            # Queue 被關閉或其他錯誤
            break
    print("[SERVER] Message bridge stopped.")

# --- 停止邏輯 ---
def stop_everything():
    """ 停止所有正在運行的 Thread 或 Process """
    global current_process, current_thread, stop_event, msg_queue
    
    # 1. 停止 Thread (Fix Version)
    if current_thread and current_thread.is_alive():
        print("[SERVER] Stopping Thread (Fix)...")
        stop_event.set()
        current_thread.join(timeout=2.0)
        current_thread = None
        stop_event.clear()

    # 2. 停止 Process (Demo/Mirror Charts)
    if current_process and current_process.is_alive():
        print("[SERVER] Terminating Process (Chart)...")
        current_process.terminate() # 強制終止子進程
        current_process.join()
        current_process = None
    
    # 3. 清理 Queue
    if msg_queue:
        msg_queue.put("STOP_BRIDGE") # 通知橋樑結束
        msg_queue = None

# --- 啟動邏輯 ---
def send_callback_thread(msg):
    """ 給 Thread 模式用的回調函式 """
    global active_connection
    if active_connection:
        try:
            active_connection.sendall(msg.encode("utf-8"))
        except:
            pass

def start_fix_thread():
    """ 啟動 Fix Version (無圖表，使用 Thread) """
    global current_thread, stop_event
    stop_everything() # 先清空
    
    print("[SERVER] Starting Fix Version (Thread)...")
    stop_event.clear()
    current_thread = threading.Thread(
        target=fix_version.main,
        kwargs={'stop_event': stop_event, 'msg_callback': send_callback_thread},
        daemon=True
    )
    current_thread.start()
    return "OK: Started Fix\n"

def start_chart_process(target_module, name):
    """ 啟動含圖表的版本 (使用 Process) """
    global current_process, msg_queue
    stop_everything() # 先清空
    
    print(f"[SERVER] Starting {name} (Process)...")
    
    # 建立通訊佇列
    msg_queue = multiprocessing.Queue()
    
    # 啟動橋樑執行緒 (負責把 Queue 的東西搬給 Unity)
    bridge = threading.Thread(target=message_bridge_loop, args=(msg_queue,), daemon=True)
    bridge.start()
    
    # 啟動子進程 (負責跑邏輯+畫圖)
    current_process = multiprocessing.Process(
        target=target_module.main_gui,
        args=(msg_queue,),
        daemon=True
    )
    current_process.start()
    return f"OK: Started {name}\n"

# --- 指令處理 ---
def handle_command(cmd: str):
    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "RUN:FIX":
        return start_fix_thread()
        
    elif cmd == "RUN:DEMO":
        return start_chart_process(demo_version, "Demo Version")
        
    elif cmd == "RUN:MIRROR":
        return start_chart_process(demo_with_mirror, "Mirror Version")
        
    elif cmd == "STOP":
        stop_everything()
        return "OK: Stopped\n"
        
    else:
        return "ERROR: UNKNOWN\n"

def client_thread(conn, addr):
    global active_connection
    print(f"[SERVER] New connection from {addr}")
    active_connection = conn

    with conn:
        buffer = b""
        while True:
            try:
                data = conn.recv(1024)
                if not data: break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    response = handle_command(line.decode("utf-8"))
                    conn.sendall(response.encode("utf-8"))
            except Exception as e:
                print(f"[SERVER] Connection error: {e}")
                break
    
    print(f"[SERVER] Client {addr} disconnected")
    active_connection = None
    # 斷線時選擇不自動停止，讓 Demo 可以繼續跑，或依需求呼叫 stop_everything()

def main():
    # 設定 multiprocessing 啟動方式 (相容性設定)
    try:
        multiprocessing.set_start_method('spawn')
    except RuntimeError:
        pass

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print("=================================================")
        print(f"[SERVER-GUI] Listening on {HOST}:{PORT}")
        print("請使用以下指令啟動，以支援 X11視窗：")
        print("ssh -Y pi@IP_ADDRESS")
        print("sudo -E python3 rpi_server_gui.py")
        print("=================================================")
        
        try:
            while True:
                conn, addr = s.accept()
                t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            print("\n[SERVER] Stopping...")
            stop_everything()

if __name__ == "__main__":
    main()