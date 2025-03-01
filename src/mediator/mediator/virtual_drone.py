from .NEDCoordinate import NEDCoordinate
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD, AgentStatus
from std_msgs.msg import Bool, UInt32

class Drone():
    status_config = {
        "IDOL": 0,
        "ARM": 1,
        "TELEOP": 2,
        "WAIT_PICK": 3,
        "ALTIUDE": 4,
    }

    def __init__(self, name: str, id: int, node: Node):
        # Init some value
        super.__init__("name")
        self.id = id
        self.state = "IDOL"
        self.name = name
        self.trajectory_setpoint_msg: TrajectorySetpoint = TrajectorySetpoint()

        # I don't know should I keep the line below or not

        # self.is_armed = False
        # self.vehicle_timestamp = 1
        # self.is_each_pre_flight_check_passed = False
        # self.start_position = NEDCoordinate(0, 0, 0)
        # self.local_position = NEDCoordinate(0, 0, 0)
        # self.heading = 0.0

        self.__arm_ready_signal: UInt32 = 0
        self.__status_signal: AgentStatus = 0
        self.__loaded_signal: UInt32 = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for px4
        self.vehicle_local_position_sub = node.create_subscription(
            VehicleLocalPosition,
            f"/{self.name}/out/vehicle_local_position",
            self.__set_vehicle_local_position,
            qos_profile
        )

        self.vehicle_status_sub = node.create_subscription(
            VehicleStatus,
            f"/{self.name}/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )
        # Subscriber for drone
        self.arm_ready_sub = node.create_subscription(
            UInt32,
            f"/{self.name}/out/arm_ready",
            self.__set_arm_ready_signal,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            AgentStatus,
            f"/{self.name}/out/status",
            self.__set_status_signal,
            qos_profile
        )

        self.loaded_sub = node.create_subscription(
            UInt32,
            f"/{self.name}/out/loaded",
            self.__set_loaded_signal,
            qos_profile
        )

        # Publishers for px4
        self.vehicle_status_pub = self.create_publisher(
            VehicleCommand,
            f"/{self.name}/in/vehicle_flight_status",
            qos_profile
        )
        # Publishers for drone
        self.arm_pub    = self.create_publisher(Bool, f"/{self.name}/in/arm", qos_profile)
        self.teleop_pub = self.create_publisher(Bool, f"/{self.name}/in/teleop", qos_profile)
        self.load_pub   = self.create_publisher(Bool, f"/{self.name}/in/load", qos_profile)
        self.drop_pub   = self.create_publisher(Bool, f"/{self.name}/in/drop", qos_profile)
        self.track_pub  = self.create_publisher(Bool, f"/{self.name}/in/track", qos_profile)

    ### Properties ###
    @property
    def received_arm_ready_signal(self) -> UInt32:
        return self.__arm_ready_signal
    
    @property
    def received_status_signal(self) -> AgentStatus:
        return self.__status_signal
    
    @property
    def received_loaded_signal(self) -> UInt32:
        return self.__loaded_signal
    
    ### Setters ###
    def __set_arm_ready_signal(self, msg : UInt32) -> None:
        self.__arm_ready_signal = msg.data
    
    def __set_status_signal(self, msg : AgentStatus) -> None:
        self.__status_signal = msg.data

    def __set_loaded_signal(self, msg : UInt32) -> None:
        self.__loaded_signal = msg.data
    
    # drone_id
    def __get_drone_id_msg(self) -> UInt32:
        uint32_msg = UInt32()
        uint32_msg.data = self.id
        return uint32_msg