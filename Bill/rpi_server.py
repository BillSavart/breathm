import socket
import threading
import time
import os
import signal

# 嘗試匯入 self_check，如果沒有該檔案可自行移除這行
try:
    from self_check import run_self_check
except ImportError:
    # 建立一個假的 check 函式以防報錯
    def run_self_check(): return True

# [重要] 匯入剛剛修改過的 fix_version
import fix_version 

# 設定為 0.0.0.0 代表接受所有來源的連線
HOST = "0.0.0.0" 
PORT = 5005

# 全域變數管理
breath_thread = None
stop_event = threading.Event()
active_connection = None # 用來儲存當前連線的 Socket

def send_to_unity(message):
    """
    這是一個回調函式(Callback)，會被傳入 fix_version.py。
    當 fix_version 想要傳資料時，會呼叫這個函式。
    """
    global active_connection
    if active_connection:
        try:
            # 發送數據給 Unity
            active_connection.sendall(message.encode("utf-8"))
            # print(f"[DEBUG] Sent to Unity: {message.strip()}")
        except Exception as e:
            print(f"[SERVER] Send Error: {e}")
            active_connection = None

def handle_command(cmd: str):
    global breath_thread, stop_event

    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "ACTIVATE":
        # 檢查是否已經在執行中
        if breath_thread is None or not breath_thread.is_alive():
            print("[SERVER] Starting breath logic...")
            stop_event.clear()
            
            # 啟動執行緒，並傳入停止事件與回調函式
            breath_thread = threading.Thread(
                target=fix_version.main, 
                kwargs={'stop_event': stop_event, 'msg_callback': send_to_unity},
                daemon=True
            )
            breath_thread.start()
            return "OK: ACTIVATE\n"
        else:
            return "INFO: Already running\n"

    elif cmd == "DEACTIVATE":
        if breath_thread and breath_thread.is_alive():
            print("[SERVER] Stopping breath logic...")
            stop_event.set() # 通知執行緒該停了
            breath_thread.join(timeout=2.0) # 等待它結束
            breath_thread = None
            return "OK: DEACTIVATE\n"
        else:
            return "INFO: Not running\n"
    else:
        return "ERROR: UNKNOWN_COMMAND\n"


def client_thread(conn, addr):
    global active_connection
    print(f"[SERVER] New connection from {addr}")
    
    # 綁定這個連線為當前活躍連線
    active_connection = conn

    with conn:
        buffer = b""
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    print(f"[SERVER] Client {addr} disconnected")
                    break
                buffer += data
                
                # 處理黏包 (Packet sticking)
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    response = handle_command(line.decode("utf-8"))
                    conn.sendall(response.encode("utf-8"))
            
            except ConnectionResetError:
                print(f"[SERVER] Connection reset by {addr}")
                break
            except Exception as e:
                print(f"[SERVER] Connection error: {e}")
                break
    
    # 連線結束後清理
    if active_connection == conn:
        active_connection = None


def main():
    # 執行自我檢查
    if not run_self_check():
        print("[SERVER] Self-check fails. System terminates")
        return

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # 允許 Port 重用，避免程式重開時顯示 Address already in use
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Listening on {HOST}:{PORT}")
        
        try:
            while True:
                conn, addr = s.accept()
                # 為每個連線開啟一個處理執行緒
                t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            print("\n[SERVER] Server stopping...")

if __name__ == "__main__":
    main()