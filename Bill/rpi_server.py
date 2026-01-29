import socket
import threading
import time
import os

# --- 匯入剛剛建立的三個純邏輯檔案 ---
import fix_version
import demo_version
import demo_with_mirror

HOST = "0.0.0.0" 
PORT = 5005

current_thread = None
stop_event = threading.Event()
active_connection = None 

def send_to_unity(message):
    global active_connection
    if active_connection:
        try:
            active_connection.sendall(message.encode("utf-8"))
        except Exception as e:
            print(f"[SERVER] Send Error: {e}")
            active_connection = None

def stop_current_script():
    global current_thread, stop_event
    if current_thread and current_thread.is_alive():
        print("[SERVER] Stopping script...")
        stop_event.set()
        current_thread.join(timeout=2.0)
    stop_event.clear()
    current_thread = None

def start_script(target_module, name):
    global current_thread, stop_event
    stop_current_script() # 先停止舊的
    
    print(f"[SERVER] Starting {name}...")
    # 這裡會統一呼叫 target_module.main()
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

    if cmd == "RUN:FIX":
        return start_script(fix_version, "Fix Version")
    elif cmd == "RUN:DEMO":
        return start_script(demo_version, "Demo Version")
    elif cmd == "RUN:MIRROR":
        return start_script(demo_with_mirror, "Mirror Version")
    elif cmd == "STOP":
        stop_current_script()
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
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[SERVER] Ready on {HOST}:{PORT}")
        try:
            while True:
                conn, addr = s.accept()
                t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            stop_current_script()

if __name__ == "__main__":
    main()