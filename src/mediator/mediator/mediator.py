import yaml
import rclpy
from .NEDCoordinate import NEDCoordinate
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)


class drone:
    status_config = {
        "IDOL": 0,
        "ARM": 1,
        "TELEOP": 2,
        "WAIT_PICK": 3,
        "ALTIUDE": 4,
    }

    def __init__(self, name: str, id: int, node: Node):
        # Init some value
        self.id = id
        self.state = "IDOL"
        self.name = name
        self.node = node
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
        self.vehicle_local_position_sub = self.node.create_subscription(
            VehicleLocalPosition,
            f"/{self.name}/out/vehicle_local_position",
            self.__set_vehicle_local_position,
            qos_profile)

        self.vehicle_status_sub = self.node.create_subscription(
            VehicleStatus,
            f"/{self.name}/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )

        # Publishers
        self.vehicle_command_pub = self.node.create_publisher(
            VehicleCommand,
            f"/{self.name}/in/vehicle_command",
            qos_profile
        )

        self.offboard_control_mode_pub = self.node.create_publisher(
            OffboardControlMode,
            f"/{self.name}/in/offboard_control_mode",
            qos_profile
        )

        self.trajectory_setpoint_pub = self.node.create_publisher(
            TrajectorySetpoint,
            f"/{self.name}/in/trajectory_setpoint",
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
        match self.state:
            case "INIT":

                self.get_logger().info(f"Armed status: {self.is_armed}")
                self.get_logger().info(
                    f"Vehicle timestamp: {self.vehicle_timestamp}")
                self.get_logger().info(
                    f"Preflight checks passed: {self.is_each_pre_flight_check_passed}")

                if self.is_each_pre_flight_check_passed:
                    self.get_logger().info("Drone is ready to arm and start offboard control.")
                    self.activate_offboard_control_mode()
                    # self.arm()
                    # self.perform_takeoff()
                    self.get_logger().info("Ok")
                    # if(self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD) : self.state = "TELEOP"
                    self.state = "TELEOP"

            case "TELEOP":
                self.move_with_velocity()
                pass

            case str(x):
                self.node.get_logger().warn(f"Invalide state {x}")

            case _:
                self.node.get_logger().error("Invalide state type, exit...")
                exit(0)

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


class mediator(Node):
    status_config = {
        "IDOL": 0,
        "ARM": 1,
        "TELEOP": 2,
        "WAIT_PICK": 3,
        "ALTIUDE": 4,
    }

    def __init__(self, path: str):
        self.config = yaml.load(path)
        super.__init__("Mediator")
        self.get_logger().info("init mediator")

    def main_loop():
        pass


def main():
    pass


if __name__ == "__main__":
    main()
