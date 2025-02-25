import yaml
import rclpy
import os
from .coordinate import Coordinate
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from esp_msg.msg import ESPCMD, State, AgentStatus
from std_msgs.msg import Bool, UInt32
from mediator.constants import NUM_DRONES, NUM_BUTTONS


class Drone(Node):
    status_config = {
        "IDLE": 0,
        "ARM": 1,
        "TELEOP": 2,
        "WAIT_PICK": 3,
        "ALTITUDE": 4,
    }

    def __init__(self, name: str, id: int):
        # Init some value
        super.__init__("name")
        self.id = id
        self.state = "IDLE"
        self.name = name
        self.trajectory_setpoint_msg: TrajectorySetpoint = TrajectorySetpoint()

        self.is_armed = False
        self.vehicle_timestamp = 1
        self.is_each_pre_flight_check_passed = False
        self.start_position = Coordinate(0, 0, 0)
        self.local_position = Coordinate(0, 0, 0)
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
        self.local_position = Coordinate(
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

def init_status_flags(size: int):
    flags = [False] * (size + 1)
    flags[0] = True
    return flags

class Mediator(Node):
    def __init__(self, path: str):
        with open(path, "r") as f:
            config = yaml.load(f)

        super.__init__("Mediator")
        self.get_logger().info("init mediator")

        self.MainDrone = Drone(config["MainDronePrefix"], 1, self)
        self.SubDrones = []
        for subdrone in config["SubDronePrefixes"]:
            self.SubDrones.append(Drone(subdrone, 1, self))

        self.is_drone_online = init_status_flags(NUM_DRONES)
        self.prev_buttons = init_status_flags(NUM_BUTTONS)
        self.__teleop_control = False
        self.__magnet_control = False
        self.__drop_control = False
        self.__drone_control = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscriber
        self.create_subscription(ESPCMD, '/esp_values', self.__set_esp_values, qos_profile)
        self.create_subscription(UInt32, '/mediator/online', self.__set_is_drone_online, qos_profile)
        self.create_subscription(AgentStatus, '/mediator/status', self.__set_status, qos_profile)

        # publisher
        self.state_pub = self.create_publisher(State, f"/agent/state", qos_profile)
        self.arm_pubs = [None] * (NUM_DRONES+1)
        self.teleop_pubs = [None] * (NUM_DRONES+1)
        self.drop_pubs = [None] * (NUM_DRONES+1)
        self.track_pubs = [None] * (NUM_DRONES+1)

        for drone_id in range(1, NUM_DRONES+1):
            self.arm_pubs[drone_id] = self.create_publisher(Bool, f"/drone_{drone_id}/arm", qos_profile)
            self.teleop_pubs[drone_id] = self.create_publisher(Bool, f"/drone_{drone_id}/teleop", qos_profile)
            self.drop_pubs[drone_id] = self.create_publisher(Bool, f"/drone_{drone_id}/drop", qos_profile)
            self.track_pubs[drone_id] = self.create_publisher(Bool, f"/drone_{drone_id}/track", qos_profile)

        self.controlled_drone: Drone = self.MainDrone
        self.main_loop()

    def main_loop(self):
        while True:
            pass

    def __set_is_drone_online(self, msg: UInt32):
        drone_id = msg.data
        if not self.is_drone_online[drone_id]:
            self.logger.ori.info(f"{drone_id} is online!")
        self.is_drone_online[drone_id] = True
        if not all(self.is_drone_online):
            return
        self.logger.info("sending arming command")
        msg = Bool()
        msg.data = True
        self.arm_pubs[drone_id].publish(msg)

    def __set_status(self, msg: AgentStatus):
        drone_id = msg.drone_id

        agent_local_position = Coordinate(
            msg.point.x,
            msg.point.y,
            msg.point.z,
        )

        if(drone_id == self.__drone_control):
            msg = Bool()
            msg.data = True
            if(self.__teleop_control):
                self.teleop_pubs[drone_id].publish(True)
            if(self.__magnet_control):
                # TODO
                pass
            if(self.__drop_control):
                pass
            




    def __set_esp_values(self, msg):
        espcmd_msg = ESPCMD()
        espcmd_msg.vx = msg.vx
        espcmd_msg.vy = msg.vy
        espcmd_msg.vz = msg.vz
        espcmd_msg.yaw = msg.yaw
        espcmd_msg.buttons = msg.buttons

        if not self.prev_buttons[0] and espcmd_msg.buttons[0]:
            self.__teleop_control = True
        if not self.prev_buttons[1] and espcmd_msg.buttons[1]:
            self.__magnet_control = True
        if not self.prev_buttons[2] and espcmd_msg.buttons[2]:
            self.__drop_control = True

        for i in range(NUM_DRONES):
            if not self.prev_buttons[3+i] and espcmd_msg.buttons[3+i]:
                self.__drone_control = i
                self.prev_buttons = init_status_flags(NUM_BUTTONS)
                self.__teleop_control = False
                self.__magnet_control = False
                self.__drop_control = False

        self.prev_buttons = espcmd_msg.buttons

        
def main(args):
    rclpy.init(args)
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()


if __name__ == "__main__":
    main(None)
