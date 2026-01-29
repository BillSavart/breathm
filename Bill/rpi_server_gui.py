#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import threading
import multiprocessing
import time
import os
import signal

# --- [修正] 匯入你指定的檔名 ---
try:
    import fix_version
    import demo_version       # 修正：對應 demo_version.py
    import demo_with_mirror   # 修正：對應 demo_with_mirror.py
except ImportError as e:
    print(f"Import Error: {e}")
    print("請確認資料夾內有: fix_version.py, demo_version.py, demo_with_mirror.py")

# 設定 IP 與 Port
HOST = "0.0.0.0" 
PORT = 5005

# 全域變數
current_process = None      
current_thread = None       
stop_event = threading.Event() 
msg_queue = None            
active_connection = None    

def message_bridge_loop(queue):
    global active_connection
    print("[SERVER] Message bridge started.")
    while True:
        try:
            msg = queue.get()
            if msg == "STOP_BRIDGE": break
            if active_connection:
                try:
                    active_connection.sendall(msg.encode("utf-8"))
                except: pass
        except: break
    print("[SERVER] Message bridge stopped.")

def stop_everything():
    global current_process, current_thread, stop_event, msg_queue
    
    if current_thread and current_thread.is_alive():
        print("[SERVER] Stopping Thread (Fix)...")
        stop_event.set()
        current_thread.join(timeout=2.0)
        current_thread = None
        stop_event.clear()

    if current_process and current_process.is_alive():
        print("[SERVER] Terminating Process (Chart)...")
        current_process.terminate()
        current_process.join()
        current_process = None
    
    if msg_queue:
        msg_queue.put("STOP_BRIDGE")
        msg_queue = None

def send_callback_thread(msg):
    global active_connection
    if active_connection:
        try: active_connection.sendall(msg.encode("utf-8"))
        except: pass

def start_fix_thread():
    global current_thread, stop_event
    stop_everything()
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
    global current_process, msg_queue
    stop_everything()
    print(f"[SERVER] Starting {name} (Process)...")
    
    msg_queue = multiprocessing.Queue()
    bridge = threading.Thread(target=message_bridge_loop, args=(msg_queue,), daemon=True)
    bridge.start()
    
    # 這裡會呼叫模組內的 main_gui
    if hasattr(target_module, 'main_gui'):
        current_process = multiprocessing.Process(
            target=target_module.main_gui,
            args=(msg_queue,),
            daemon=True
        )
        current_process.start()
        return f"OK: Started {name}\n"
    else:
        print(f"Error: {name} does not have main_gui function.")
        return "ERROR: COMPATIBILITY_FAIL\n"

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
            except: break
    print(f"[SERVER] Client {addr} disconnected")
    active_connection = None

def main():
    try: multiprocessing.set_start_method('spawn')
    except RuntimeError: pass

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print("=================================================")
        print(f"[SERVER-GUI] Listening on {HOST}:{PORT}")
        print("SSH 指令: ssh -Y pi@IP")
        print("執行指令: sudo -E python3 rpi_server_gui.py")
        print("=================================================")
        
        try:
            while True:
                conn, addr = s.accept()
                t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            stop_everything()

if __name__ == "__main__":
    main()