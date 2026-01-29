import socket
import threading
import time
import os

# --- [關鍵修改] 匯入所有模式的腳本 ---
# 請確保這三個 .py 檔都在同一個資料夾下
import fix_version
import demo_version
import demo_with_mirror

# 設定 IP
HOST = "0.0.0.0" 
PORT = 5005

# 全域變數管理
current_thread = None
stop_event = threading.Event()
active_connection = None 

def send_to_unity(message):
    """ 回調函式：讓腳本可以傳送訊號給 Unity """
    global active_connection
    if active_connection:
        try:
            active_connection.sendall(message.encode("utf-8"))
        except Exception as e:
            print(f"[SERVER] Send Error: {e}")
            active_connection = None

def stop_current_script():
    """ 輔助函式：安全停止目前正在執行的腳本 """
    global current_thread, stop_event
    
    if current_thread and current_thread.is_alive():
        print("[SERVER] Stopping current script...")
        stop_event.set() # 發出停止信號
        current_thread.join(timeout=2.0) # 等待它結束
        if current_thread.is_alive():
            print("[SERVER] Warning: Thread did not stop gracefully.")
        else:
            print("[SERVER] Script stopped.")
    
    # 重置變數
    stop_event.clear()
    current_thread = None

def start_script(target_module, name):
    """ 輔助函式：啟動指定的腳本 """
    global current_thread, stop_event
    
    # 1. 先停止正在跑的
    stop_current_script()
    
    # 2. 啟動新的
    print(f"[SERVER] Starting {name}...")
    current_thread = threading.Thread(
        target=target_module.main, 
        kwargs={'stop_event': stop_event, 'msg_callback': send_to_unity},
        daemon=True
    )
    current_thread.start()
    return f"OK: Started {name}\n"

def handle_command(cmd: str):
    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    # --- [關鍵修改] 根據指令切換不同腳本 ---
    if cmd == "RUN:FIX":
        return start_script(fix_version, "Fix Version")
        
    elif cmd == "RUN:DEMO":
        return start_script(demo_version, "Demo Version")
        
    elif cmd == "RUN:MIRROR":
        return start_script(demo_with_mirror, "Demo with Mirror")
        
    elif cmd == "STOP":
        stop_current_script()
        return "OK: Stopped\n"
        
    else:
        return "ERROR: UNKNOWN_COMMAND\n"

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
    # 斷線時也可以選擇是否要自動停止腳本，這裡選擇保持執行或手動停止

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Multi-Mode Ready on {HOST}:{PORT}")
        print("Wait for commands: RUN:FIX, RUN:DEMO, RUN:MIRROR, STOP")
        
        try:
            while True:
                conn, addr = s.accept()
                t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            print("\n[SERVER] Server stopping...")
            stop_current_script()

if __name__ == "__main__":
    main()