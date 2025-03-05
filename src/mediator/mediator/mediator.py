import yaml
import rclpy
import os

from enum import Enum, auto, unique

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD
from virtual_drone import Drone
from mediator.constants import NUM_DRONES, NUM_BUTTONS


@unique
class MediatorEnum(Enum):
    """
    This is enum, auto function from imported built-in
    class enum will auto asigned unique number to the item
    """
    IDLE = auto()
    FETCHING = auto()
    ALIGNED = auto()
    ERROR = auto()


class Mediator(Node):

    def __init__(self, path: str):

        super.__init__("Mediator")
        self.get_logger().info("init mediator")

        self.MainDrone = Drone(1, self)
        self.SubDrones = []
        for i in range(2,4):
            self.SubDrones.append(Drone(i, self)) # should the id be 2 and 3???

        self.controlled_drone: Drone = self.MainDrone


        self.prev_buttons = [False] * NUM_BUTTONS
        self.__teleop_btn_signal = False
        self.__magnet_btn_signal = False
        self.__drop_btn_signal = False
        self.__current_drone = 1
        self.state = MediatorEnum.IDLE

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscriber
        self.create_subscription(ESPCMD, '/esp_values', self.__set_esp_values, qos_profile)

        self.create_timer(0.01, self.execute)

    def execute(self):
        """
        There are mediator states:
        idle:
           This state waits for any signal and then enters the fetching state.
        fetching:
           This is the fetching state.
           Drones will receive instructions one by one and wait in place.
        aligned:
           This is the aligned state.
           The sub-drones will follow the main drone.
        error:
            This is error handling state. After this state,
            the drone should return to its original state
        """
        match self.state:
            case MediatorEnum.IDLE:
                pass
                # TODO waiting for
            case MediatorEnum.FETCHING:
                # TODO this is fetching
                pass
            case MediatorEnum.ALIGNED:
                # TODO This is aligned
                pass
            case MediatorEnum.ERROR:
                # TODO This is error handling.
                pass
            case _:
                self.get_logger()\
                    .error(
                        f"wrong type, {self.state},\
                        you should use MediatorEnum to represent type"
                )
                exit(1)
        pass

    def __set_esp_values(self, msg):
        buttons = msg.buttons

        self.__teleop_btn_signal = self.prev_buttons[0] ^ buttons[0]
        self.__magnet_btn_signal = self.prev_buttons[1] ^ buttons[1]
        self.__drop_btn_signal = self.prev_buttons[2] ^ buttons[2]             

        for i in range(NUM_DRONES):
            if self.prev_buttons[3+i] ^ buttons[3+i]:
                self.__current_drone = i + 1
                # self.prev_buttons = [False] * NUM_BUTTONS
                self.__teleop_btn_signal = False
                self.__magnet_btn_signal = False
                self.__drop_btn_signal = False

        # self.execute()  # TODO Is there a better way

        self.prev_buttons = buttons


def main(args):
    rclpy.init(args)
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()


if __name__ == "__main__":
    main(None)
