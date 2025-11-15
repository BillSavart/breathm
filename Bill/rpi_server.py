# rpi_server.py
import socket
import threading
import subprocess

HOST = "192.168.0.201"
PORT = 5005

def handle_command(cmd: str):
    cmd = cmd.strip()
    print(f"[SERVER] Received command: {cmd}")

    if cmd == "FEED_PET":
        # 直接呼叫某個 Python 函式或腳本
        subprocess.Popen(["python3", "helloworld.py"])
        return "OK: FEED_PET\n"

    elif cmd == "TOGGLE_LIGHT":
        subprocess.Popen(["python3", "helloworld.py"])
        return "OK: TOGGLE_LIGHT\n"

    elif cmd.startswith("START_SCRIPT:"):
        script_name = cmd.split(":", 1)[1]
        subprocess.Popen(["python3", f"{script_name}.py"])
        return f"OK: START_SCRIPT {script_name}\n"

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
            # 以換行符號分割指令
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
