import yaml
import rclpy
import os
from .NEDCoordinate import NEDCoordinate
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD
from virtual_drone import Drone

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
        self.main_loop()

    def main_loop(self):
        while True:
            pass


def main(args):
    rclpy.init(args)
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()


if __name__ == "__main__":
    main(None)
