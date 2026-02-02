import socket
import threading
import subprocess
import os
import signal
import sys
from self_check import run_self_check

HOST = "0.0.0.0"
PORT = 5005

breathm_process = None

# 1. 取得絕對路徑，確保不管在哪執行都能找到 fix_version.py
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_PATH = os.path.join(CURRENT_DIR, "demo_version.py")

def monitor_process_output(proc, conn):
    """
    持續讀取子程序的 stdout (包含 stderr)，如果有包含 SYNC_ 的關鍵字，
    就透過 socket 傳送給 Unity。
    """
    try:
        # 逐行讀取輸出
        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            line = line.strip()
            
            # [關鍵] 印出所有 Log，這樣你才看得到它有沒有在跑，或有沒有報錯
            print(f"[SCRIPT Log] {line}") 

            if "SYNC_" in line:
                try:
                    msg = line + "\n"
                    conn.sendall(msg.encode("utf-8"))
                except Exception as e:
                    print(f"[SERVER] Failed to send sync data: {e}")
                    break
    except Exception as e:
        print(f"[SERVER] Monitor thread error: {e}")
    finally:
        print("[SERVER] Process monitor ended")

def handle_command(cmd: str, conn):
    global breathm_process

    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "ACTIVATE":
        if breathm_process is None or breathm_process.poll() is not None:
            print(f"[SERVER] Attempting to start script: {SCRIPT_PATH}")
            
            try:
                # 2. 啟動指令修正：
                # sys.executable : 確保使用目前的 Python 環境 (venv)
                # "-u"           : 強制不緩衝，讓 print 馬上顯示
                # stderr=subprocess.STDOUT : 讓錯誤訊息也顯示在 Log 裡
                breathm_process = subprocess.Popen(
                    [sys.executable, "-u", SCRIPT_PATH], 
                    stdout=subprocess.PIPE, 
                    stderr=subprocess.STDOUT, 
                    text=True,
                    bufsize=1 
                )
                
                t = threading.Thread(target=monitor_process_output, args=(breathm_process, conn), daemon=True)
                t.start()
                
                return "OK: ACTIVATE\n"
            except Exception as e:
                print(f"[SERVER] Failed to start process: {e}")
                return f"ERROR: Launch failed {e}\n"
        else:
            return "INFO: Script already running\n"

    elif cmd == "DEACTIVATE":
        if breathm_process is not None and breathm_process.poll() is None:
            print("[SERVER] Killing process...")
            os.kill(breathm_process.pid, signal.SIGTERM)
            breathm_process = None
            return "OK: DEACTIVATE\n"
        else:
            return "INFO: Script is NOT running\n"
    else:
        return "ERROR: UNKNOWN_COMMAND\n"

def client_thread(conn, addr):
    print(f"[SERVER] New connection from {addr}")
    with conn:
        buffer = b""
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    print(f"[SERVER] Client {addr} disconnected")
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    response = handle_command(line.decode("utf-8"), conn)
                    conn.sendall(response.encode("utf-8"))
            except ConnectionResetError:
                print(f"[SERVER] Connection reset by {addr}")
                break
            except Exception as e:
                print(f"[SERVER] Error: {e}")
                break

def main():
    if not run_self_check():
        print("[SERVER] Self-check fails. System terminates")
        return

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
