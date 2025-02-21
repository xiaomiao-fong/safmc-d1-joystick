import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import serial
import threading
import json
import time

import rclpy.publisher
from esp_msg.msg import ESPCMD
from .data_processer import DataProcessor

MAX_SPEED = 2.0  # 限制最大速度 2 m/s

# 全域變數，儲存最新的數據

class ESPTransmitter(Node):

    def __init__(self):
        super().__init__("ESP_Transmitter")
        self.get_logger().info("Start transmitting")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]  # 根據需求修改
        self.processor = DataProcessor(self.ports)

        self.create_timer(0.01, self.read_esp_data)
        self.esp_vel_pub : rclpy.publisher.Publisher = self.create_publisher(ESPCMD, "/esp_vel", qos_profile)
        
    def data_handler(self, output_data):
        
        if output_data is None : return

        final_vx = output_data[0]
        final_vy = output_data[1]
        final_vz = output_data[2]
        final_yaw_rate = output_data[3]

        print(f"飛行指令: vx={final_vx:5.2f}, vy={final_vy:5.2f}, vz={final_vz:5.2f}, yaw_rate={final_yaw_rate:5.2f}", end = '\r')

        espcmd_msg = ESPCMD()
        espcmd_msg.vx = final_vx
        espcmd_msg.vy = final_vy
        espcmd_msg.vz = final_vz
        espcmd_msg.yaw = final_yaw_rate

        self.esp_vel_pub.publish(espcmd_msg)
 
    def read_esp_data(self):
        self.processor.process_data()
        self.data_handler(self.processor.output_data)
    
def main(args = None):
    rclpy.init(args=args)
    node = ESPTransmitter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
