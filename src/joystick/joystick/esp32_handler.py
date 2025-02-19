import serial
import json
import threading
import time

class ESP32Handler:
    def __init__(self, port):
        self.port = port
        self.ser = self.open_serial()
        self.latest_data = None  # 存儲最新的數據
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

    def open_serial(self):
        try:
            return serial.Serial(port=self.port, baudrate=115200, timeout=0.1)
        except Exception as e:
            print(f"無法開啟串列埠 {self.port}: {e}")
            return None

    def read_serial(self):
        if self.ser is None:
            print("bye bye")
            return
        
        while True:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            json_data = json.loads(line)
                            self.latest_data = json_data  # 更新最新數據
                        except json.JSONDecodeError:
                            print(f"❌ JSON 解析失敗: {line}")
            except Exception as e:
                print(f"[錯誤] 串列讀取失敗: {e}")
                break

    def get_latest_data(self):
        return self.latest_data  # 提供獲取最新數據的方法

def create_handler(port):
    return ESP32Handler(port)
