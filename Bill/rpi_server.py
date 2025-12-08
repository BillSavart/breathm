import socket
import threading
import subprocess
import os
import signal
from self_check import run_self_check

HOST = "172.20.10.4"
PORT = 5005

breathm_process = None

def handle_command(cmd: str):
    global breathm_process

    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "ACTIVATE":
        if breathm_process is None or breathm_process.poll() is not None:
            breathm_process = subprocess.Popen(["python3", "demo_version.py"])
            return "OK: ACTIVATE\n"
        else:
            return "INFO: demo_version.py already running\n"

    elif cmd == "DEACTIVATE":
        if breathm_process is not None and breathm_process.poll() is None:
            os.kill(breathm_process.pid, signal.SIGTERM)
            breathm_process = None
            return "OK: DEACTIVATE\n"
        else:
            return "INFO: demo_version.py is NOT running\n"
    else:
        return "ERROR: UNKNOWN_COMMAND\n"


def client_thread(conn, addr):
    print(f"[SERVER] New connection from {addr}")
    with conn:
        buffer = b""
        while True:
            data = conn.recv(1024)
            if not data:
                print(f"[SERVER] Client {addr} disconnected")
                break
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                response = handle_command(line.decode("utf-8"))
                conn.sendall(response.encode("utf-8"))


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
