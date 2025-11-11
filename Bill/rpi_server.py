#!/usr/bin/env python3
import socket
import subprocess
import os

# === Python 檔所在的資料夾 ===
SCRIPT_DIR = "/home/imlab/Desktop/breathm/Bill"  # 改成你的實際路徑

# === 指令字串 → 對應的 python 檔名 ===
# 這裡照你自己需要改
COMMAND_TO_SCRIPT = {
    "SCRIPT1": "helloworld.py",
    "SCRIPT2": "script2.py",
    "LIGHT_ON": "light_on.py",
    "LIGHT_OFF": "light_off.py",
    # 你可以一直加： "指令字串": "檔名.py"
}

# === TCP Server 設定 ===
HOST = ""        # 空字串 = 綁定所有網路介面
PORT = 5005      # 要跟 Unity 那邊設定的一樣

def run_python_script(script_filename: str) -> str:
    """在 SCRIPT_DIR 底下執行指定的 python 檔，回傳簡單文字結果。"""
    script_path = os.path.join(SCRIPT_DIR, script_filename)

    if not os.path.isfile(script_path):
        msg = f"Script not found: {script_path}"
        print(msg)
        return msg

    try:
        # 如果腳本會跑很久、你不想等，可以改用 subprocess.Popen
        result = subprocess.run(
            ["python3", script_path],
            capture_output=True,
            text=True
        )
        # 把 stdout / stderr 印在 server 端，也回一點 summary 給 Unity
        print(f"Ran {script_filename}, return code = {result.returncode}")
        if result.stdout:
            print("STDOUT:")
            print(result.stdout)
        if result.stderr:
            print("STDERR:")
            print(result.stderr)

        if result.returncode == 0:
            # 成功
            return f"OK: {script_filename} finished."
        else:
            # 執行有錯
            return f"ERROR: {script_filename} exit code {result.returncode}"
    except Exception as e:
        msg = f"Exception when running {script_filename}: {e}"
        print(msg)
        return msg

def handle_command(cmd: str) -> str:
    """解析 Unity 傳來的指令字串，決定要跑哪個腳本。"""
    raw_cmd = cmd.strip()
    cmd_upper = raw_cmd.upper()
    print(f"Received command: {raw_cmd}")

    if cmd_upper in COMMAND_TO_SCRIPT:
        script_filename = COMMAND_TO_SCRIPT[cmd_upper]
        return run_python_script(script_filename)
    else:
        msg = f"Unknown command: {raw_cmd}"
        print(msg)
        return msg

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Server started. Listening on port {PORT}...")

    try:
        while True:
            conn, addr = s.accept()
            print("Connected by", addr)

            data = conn.recv(1024)
            if not data:
                conn.close()
                continue

            cmd = data.decode("utf-8")
            response = handle_command(cmd)

            # 回覆給 Unity
            try:
                conn.sendall(response.encode("utf-8"))
            except Exception as e:
                print(f"Error sending response: {e}")

            conn.close()
    except KeyboardInterrupt:
        print("Stopping server...")
    finally:
        s.close()

if __name__ == "__main__":
    main()
