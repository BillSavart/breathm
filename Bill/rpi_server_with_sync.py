import socket
import threading
import subprocess
import os
import signal
import time

HOST = "172.20.10.4" # 請確認這是不是你 RPi 的當前 IP
PORT = 5005

breathm_process = None
current_client_conn = None # 用來存儲當前連線，以便回傳動畫訊號

def monitor_process_output(proc, conn):
    """
    這個函式會持續讀取 fix_version.py 的 print 輸出
    並將關鍵字傳回給 Unity
    """
    try:
        # 逐行讀取 stdout
        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            
            clean_line = line.strip()
            print(f"[LOG from script]: {clean_line}")
            
            # 如果讀到的字串包含 ANIM 指令，回傳給 Unity
            if "ANIM:" in clean_line:
                try:
                    conn.sendall((clean_line + "\n").encode("utf-8"))
                except:
                    break # 連線斷了就停止發送
    except Exception as e:
        print(f"[SERVER] Monitor thread error: {e}")

def handle_command(cmd: str, conn):
    global breathm_process, current_client_conn
    current_client_conn = conn

    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "RUN:FIX":
        # 如果已經有在跑，先殺掉
        if breathm_process is not None and breathm_process.poll() is None:
            os.kill(breathm_process.pid, signal.SIGTERM)
            breathm_process = None
            time.sleep(0.5)

        # 啟動 fix_version.py，並開啟 stdout 管道 (PIPE)
        # unbuffered (-u) 確保 Python 不要暫存輸出
        breathm_process = subprocess.Popen(
            ["python3", "-u", "fix_version.py"], 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 啟動一個執行緒來監聽輸出並回傳給 Unity
        t = threading.Thread(target=monitor_process_output, args=(breathm_process, conn), daemon=True)
        t.start()

        return "OK: FIX_STARTED\n"

    elif cmd == "STOP":
        if breathm_process is not None:
            if breathm_process.poll() is None:
                os.kill(breathm_process.pid, signal.SIGTERM)
            breathm_process = None
            return "OK: STOPPED\n"
        else:
            return "INFO: NOTHING_RUNNING\n"
    
    else:
        return "ERROR: UNKNOWN\n"

def client_thread(conn, addr):
    print(f"[SERVER] Connected by {addr}")
    with conn:
        buffer = b""
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    decoded_line = line.decode("utf-8").strip()
                    if decoded_line:
                        response = handle_command(decoded_line, conn)
                        conn.sendall(response.encode("utf-8"))
            except Exception as e:
                print(f"[SERVER] Error: {e}")
                break
    print(f"[SERVER] Disconnected {addr}")

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Listening on {HOST}:{PORT}")
        while True:
            conn, addr = s.accept()
            t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
            t.start()

if __name__ == "__main__":
    main()