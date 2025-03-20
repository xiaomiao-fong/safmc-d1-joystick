import yaml
import rclpy
import os
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD, AgentStatus
from .virtual_drone import Drone
from mediator.constants import NUM_DRONES, NUM_BUTTONS

class Mediator(Node):

    def __init__(self):

        super().__init__("Mediator")
        self.get_logger().info("init mediator")

        self.ids = [1,7,8]
        self.drones = [Drone(self.ids[i], self) for i in range(3)]

        self.prev_buttons = [False] * NUM_BUTTONS
        self.__teleop_btn_signal = False
        self.__magnet_btn_signal = False
        self.__drop_btn_signal = False
        self.__current_drone = -1
        self.__next_drone = -1

        self.__first = True
        self.__vel = [1500, 1500, 1500, 1500]

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscriber
        self.create_subscription(ESPCMD, '/esp_vel', self.__set_esp_values, qos_profile)

    def execute(self):

        print(f"drone_{self.ids[0]} : {' on' if self.__current_drone == 0 else 'off'}, drone_{self.ids[1]} : {' on' if self.__current_drone == 1 else 'off'}, drone_{self.ids[2]} : {' on' if self.__current_drone == 2 else 'off'}, vel: {self.__vel}")

        if self.__current_drone == -1 and self.__next_drone == -1 : return

        if self.__current_drone != self.__next_drone and self.__current_drone != -1:
            current_drone = self.drones[self.__current_drone]
            current_drone.idle()
            print(self.__current_drone, "idle")
            self.__current_drone = -1
        
        if self.__current_drone == -1 and not self.__next_drone == -1:
            current_drone = self.drones[self.__next_drone]
            current_drone.arm()
            self.__current_drone = self.__next_drone
            return
        
        if self.__current_drone == -1 : return

        current_drone = self.drones[self.__current_drone]

        if self.__magnet_btn_signal:
            current_drone.load()

        if self.__drop_btn_signal:
            current_drone.drop()

        if current_drone.is_armed:
            current_drone.teleop(self.__vel)
            return
            
    def vel_mapping(self, vels):
        # vels : [vx, vy, vz, yawRate]
        # velocities : AETR

        velocities = [1500, 1500, 1500, 1500]
        velocities[0] += vels[1]*500
        velocities[1] += vels[0]*500
        velocities[2] += vels[2]*500
        velocities[3] += vels[3]*500

        return velocities
        

    def __set_esp_values(self, msg):
        self.__vel = self.vel_mapping([msg.vx, msg.vy, msg.vz, msg.yaw])
        buttons = msg.buttons

        if self.__first :
            self.__first = False
            self.prev_buttons = buttons
            print(self.__current_drone)
            return

        self.__next_drone = self.__current_drone

        self.__teleop_btn_signal = self.prev_buttons[0] ^ buttons[0]
        self.__magnet_btn_signal = self.prev_buttons[1] ^ buttons[1]
        self.__drop_btn_signal   = self.prev_buttons[2] ^ buttons[2]             

        for i in range(NUM_DRONES):
            if self.prev_buttons[3+i] ^ buttons[3+i]:
                self.__next_drone = i if self.__current_drone != i else -1
                # self.prev_buttons = [False] * NUM_BUTTONS
                self.__teleop_btn_signal = False
                self.__magnet_btn_signal = False
                self.__drop_btn_signal = False

        # print(self.__next_drone)
        self.execute() # TODO Is there a better way

        self.prev_buttons = buttons


def main():
    rclpy.init()
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
