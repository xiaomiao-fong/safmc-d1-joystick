import time
import json
from .esp32_handler import create_handler

class DataProcessor:
    def __init__(self, ports):
        self.handlers = {port: create_handler(port) for port in ports}
        self.latest_data = {"left": None, "right": None}
        self.follow_system = False
        self.magnetic_system = False
        self.descend_system = False
        self.drop_payload_system = False
        self.left_data = None
        self.right_data = None
        self.output_data = (0.0, 0.0, 0.0, 0.0, [False] * 6)
    
    def fetch_latest_data(self):
        for port, handler in self.handlers.items():
            data = handler.get_latest_data()
            if data:
                hand = data.get("hand", "unknown")
                if hand in self.latest_data:
                    self.latest_data[hand] = data
    
    def process_data(self):
        
        self.fetch_latest_data()
        
        self.left_data = self.latest_data.get("left")
        self.right_data = self.latest_data.get("right")
        
        vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
        buttons = [False] * 6
        
        output_messages = []
        
        if self.right_data:
            buttons[0] = self.right_data.get("button2", False)
            buttons[1] = self.right_data.get("button3", False)
            buttons[2] = self.right_data.get("button4", False)
            pitch = self.right_data.get("pitch", 0.0)
            roll = self.right_data.get("roll", 0.0)
            
            # self.follow_system = button3
            # self.magnetic_system = button4
            vx = self.custom_angle_to_speed(roll)
            vy = self.custom_angle_to_speed(pitch)
            #output_messages.append(f"跟隨系統: {self.follow_system}, 磁吸系統: {self.magnetic_system}")
        
        if self.left_data:
            buttons[3] = self.right_data.get("button2", False)
            buttons[4] = self.left_data.get("button3", False)
            buttons[5] = self.left_data.get("button4", False)
            pitch = self.left_data.get("pitch", 0.0)
            roll = self.left_data.get("roll", 0.0)
            
            # self.descend_system = button3
            # self.drop_payload_system = button4
            yaw_rate = self.custom_angle_to_speed(pitch)
            vz = self.custom_angle_to_speed(roll)
            #output_messages.append(f"下降系統: {self.descend_system}, 丟payload系統: {self.drop_payload_system}")
        
        self.output_data = (vx, vy, vz, yaw_rate, buttons)
        output_messages.append(f"飛行指令: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, yaw_rate={yaw_rate:.3f}")
        
        #print("\n".join(output_messages))
        time.sleep(0.02)  # 控制更新頻率，提高即時性

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def custom_angle_to_speed(self, angle: float) -> float:
        """
        分段映射:
        1) -15° ~ 15° => 0
        2) 15° ~ 30° => 0 ~ 1  (線性映射)
        3) 30° ~ 60° => 1     (飽和)
        """
        angle = self.clamp(angle, -60.0, 60.0)
        sign = 1.0 if angle >= 0 else -1.0
        abs_angle = abs(angle)

        if abs_angle <= 15.0:
            speed = 0.0
        elif abs_angle >= 30.0:
            speed = 1.0
        else:
            speed = (abs_angle - 15.0) / (30.0 - 15.0)
        
        return sign * speed
