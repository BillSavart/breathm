import socket
import threading
import subprocess
import os
import signal

HOST = "192.168.0.201"
PORT = 5005

helloworld_process = None

def handle_command(cmd: str):
    global helloworld_process

    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "FEED_PET":
        if helloworld_process is None or helloworld_process.poll() is not None:
            helloworld_process = subprocess.Popen(["python3", "helloworld.py"])
            return "OK: FEED_PET\n"
        else:
            return "INFO: helloworld.py already running\n"

    elif cmd == "TOGGLE_LIGHT":
        if helloworld_process is not None and helloworld_process.poll() is None:
            os.kill(helloworld_process.pid, signal.SIGTERM)
            helloworld_process = None
            return "OK: TOGGLE_LIGHT\n"
        else:
            return "INFO: helloworld.py is NOT running\n"
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
