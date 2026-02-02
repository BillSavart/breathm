import socket
import time
import math

# --- 設定區 ---
UNITY_IP = '192.168.50.31'  # 請改成你的 Windows IP
UNITY_PORT = 25001
# -------------

print(f"連線到 {UNITY_IP}:{UNITY_PORT}...")

try:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((UNITY_IP, UNITY_PORT))
    print("✅ 連線成功！開始發送時間軸訊號...")

    # 用來模擬呼吸的變數
    angle = 0.0 

    while True:
        # 使用 Sin 波來模擬呼吸的自然曲線 (0 -> 1 -> 0)
        # map sin(-1~1) to (0~1)
        breath_value = (math.sin(angle) + 1) / 2
        
        # 格式化成字串，取小數點後3位
        msg = f"{breath_value:.3f}"
        
        try:
            client.sendall(msg.encode('utf-8'))
            # print(f"數值: {msg}", end='\r') # 顯示目前數值
        except BrokenPipeError:
            print("❌ 連線中斷，嘗試重連...")
            client.close()
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((UNITY_IP, UNITY_PORT))

        angle += 0.05  # 控制呼吸速度 (數字越小越慢)
        time.sleep(0.05) # 發送頻率 (約 20Hz)

except KeyboardInterrupt:
    print("\n停止測試")
    client.close()
