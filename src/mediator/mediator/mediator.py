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


class Drone(Node):
    status_config = {
        "IDOL": 0,
        "ARM": 1,
        "TELEOP": 2,
        "WAIT_PICK": 3,
        "ALTIUDE": 4,
    }

    def __init__(self, name: str, id: int):
        # Init some value
        super.__init__("name")
        self.id = id
        self.state = "IDOL"
        self.name = name
        self.trajectory_setpoint_msg: TrajectorySetpoint = TrajectorySetpoint()

        self.is_armed = False
        self.vehicle_timestamp = 1
        self.is_each_pre_flight_check_passed = False
        self.start_position = NEDCoordinate(0, 0, 0)
        self.local_position = NEDCoordinate(0, 0, 0)
        self.heading = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"/{self.name}/out/vehicle_local_position",
            self.__set_vehicle_local_position,
            qos_profile)

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f"/{self.name}/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )

        # Publishers for node on the drone
        self.vehicle_status_pub = self.create_publisher(
            VehicleCommand,
            f"/{self.name}/in/vehicle_flight_status",
            qos_profile
        )

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self.is_each_pre_flight_check_passed = \
            vehicle_status_msg.pre_flight_checks_pass
        self.vehicle_timestamp = vehicle_status_msg.timestamp
        self.nav_state = vehicle_status_msg.nav_state
        self.is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def __set_vehicle_local_position(
        self,
        vehicle_local_position_msg: VehicleLocalPosition
    ):
        self.heading = vehicle_local_position_msg.heading
        self.local_position = NEDCoordinate(
            x=vehicle_local_position_msg.x,
            y=vehicle_local_position_msg.y,
            z=vehicle_local_position_msg.z
        )

    def execute_state(self):
        while True:
            pass


    def set_trajectory(self, velocity, yawspeed, position):
        match velocity:
            case list(p):
                self.trajectory_setpoint_msg.position[0] = p[0]
                self.trajectory_setpoint_msg.position[1] = p[1]
                self.trajectory_setpoint_msg.position[2] = p[2]
            case _:
                self.trajectory_setpoint_msg.position[0] = None
                self.trajectory_setpoint_msg.position[1] = None
                self.trajectory_setpoint_msg.position[2] = None

        match velocity:
            case list(v):
                self.trajectory_setpoint_msg.velocity[0] = v[0]
                self.trajectory_setpoint_msg.velocity[1] = v[1]
                self.trajectory_setpoint_msg.velocity[2] = v[2]
            case _:
                self.trajectory_setpoint_msg.velocity[0] = None
                self.trajectory_setpoint_msg.velocity[1] = None
                self.trajectory_setpoint_msg.velocity[2] = None

        match yawspeed:
            case float(f):
                self.trajectory_setpoint_msg.yawspeed = f
            case _:
                self.trajectory_setpoint_msg = None

    def move_with_velocity(self):
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.vehicle_timestamp

        trajectory_setpoint_msg.velocity[0] = 0.05
        trajectory_setpoint_msg.velocity[1] = 0.0
        trajectory_setpoint_msg.velocity[2] = 0.0
        trajectory_setpoint_msg.yawspeed = 0.0

        trajectory_setpoint_msg.position[0] = None
        trajectory_setpoint_msg.position[1] = None
        trajectory_setpoint_msg.position[2] = None
        trajectory_setpoint_msg.yaw = self.heading

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)


class Mediator(Node):
    def __init__(self, path: str):
        with open(path, "r") as f:
            config = yaml.load(f)

        super.__init__("Mediator")
        self.get_logger().info("init mediator")

        self.MainDrone = Drone(config["MainDronePrefix"], 1, self)
        self.SubDrones = []

        self.prev_buttons = [False] * 6
        self.takeoff_state = False
        self.agent_state = 0     # 0->A, 1->B, 2->C, 3->D
        self.control_state = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(ESPCMD, '/esp_vel', self.__set_is_drone_online, qos_profile)

        for subdrone in config["SubDronePrefixes"]:
            self.SubDrones.append(Drone(subdrone, 1, self))

        self.controlled_drone: Drone = self.MainDrone
        self.main_loop()

    def main_loop(self):
        while True:
            pass

    def __set_esp_vel(self, msg):
        espcmd_msg = ESPCMD()
        espcmd_msg.vx = msg.vx
        espcmd_msg.vy = msg.vy
        espcmd_msg.vz = msg.vz
        espcmd_msg.yaw = msg.yaw
        espcmd_msg.buttons = msg.buttons

        if not self.prev_buttons[0] and espcmd_msg.buttons[0]:
            self.takeoff_state = 1
        if not self.prev_buttons[1] and espcmd_msg.buttons[1]:
            self.agent_state = (self.agent_state + 1) % 4
        if not self.prev_buttons[2] and espcmd_msg.buttons[2]:
            self.control_state = (self.control_state + 1) % 3



def main(args):
    rclpy.init(args)
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()


if __name__ == "__main__":
    main(None)
