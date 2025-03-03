import yaml
import rclpy
import os
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD
from virtual_drone import Drone
from mediator.constants import NUM_DRONES, NUM_BUTTONS

class Mediator(Node):

    def __init__(self, path: str):
        with open(path, "r") as f:
            config = yaml.load(f)

        super.__init__("Mediator")
        self.get_logger().info("init mediator")

        self.MainDrone = Drone(config["MainDronePrefix"], 1, self)
        self.SubDrones = []
        for subdrone in config["SubDronePrefixes"]:
            self.SubDrones.append(Drone(subdrone, 1, self)) # should the id be 2 and 3???

        self.controlled_drone: Drone = self.MainDrone
        self.execute()

        self.create_timer(0.03, self.execute)

        self.prev_buttons = [False] * NUM_BUTTONS
        self.__teleop_control = False
        self.__magnet_control = False
        self.__drop_control = False
        self.__drone_control = 0
        self.__vx = 0.0
        self.__vy = 0.0
        self.__vz = 0.0
        self.__yaw = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscriber
        self.create_subscription(ESPCMD, '/esp_values', self.__set_esp_values, qos_profile)

    def execute(self):
        pass

    def __set_esp_values(self, msg):
        self.__vx = msg.vx
        self.__vy = msg.vy
        self.__vz = msg.vz
        self.__yaw = msg.yaw
        buttons = msg.buttons

        if self.prev_buttons[0] ^ buttons[0]:
            self.__teleop_control = True
        if self.prev_buttons[1] ^ buttons[1]:
            self.__magnet_control = True
        if self.prev_buttons[2] ^ buttons[2]:
            self.__drop_control = True

        for i in range(NUM_DRONES):
            if self.prev_buttons[3+i] ^ buttons[3+i]:
                self.__drone_control = i
                # self.prev_buttons = [False] * NUM_BUTTONS
                self.__teleop_control = False
                self.__magnet_control = False
                self.__drop_control = False

        self.prev_buttons = buttons


def main(args):
    rclpy.init(args)
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()


if __name__ == "__main__":
    main(None)
