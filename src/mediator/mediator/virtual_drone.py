from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD, AgentStatus
from std_msgs.msg import Bool, UInt32

class Drone():
    def __str__(self):
        return self.drone_prefix

    def __init__(self, id: int, node: Node):
        # Init some value
        self.id = id
        self.mediator_node = node
        self.drone_prefix = f"/drone_{self.id}"
        self.px4_prefix = f"/px4_{self.id}"

        self.__arm_ready_signal: bool = False
        self.__status_signal: AgentStatus = None
        self.__loaded_signal: bool = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber for px4
        self.vehicle_status_sub = node.create_subscription(
            VehicleStatus,
            f"{self.px4_prefix}/fmu/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )
        # Subscriber for drone
        self.arm_ready_sub = node.create_subscription(
            UInt32,
            f"{self.drone_prefix}/out/arm_ready",
            self.__set_arm_ready_signal,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            AgentStatus,
            f"{self.drone_prefix}/out/status",
            self.__set_status,
            qos_profile
        )

        self.loaded_sub = node.create_subscription(
            UInt32,
            f"{self.drone_prefix}/out/loaded",
            self.__set_loaded_signal,
            qos_profile
        )

        # Publishers for drone
        self.__arm_pub    = node.create_publisher(Bool, f"{self.drone_prefix}/in/arm", qos_profile)
        self.__teleop_pub = node.create_publisher(Bool, f"{self.drone_prefix}/in/teleop", qos_profile)
        self.__load_pub   = node.create_publisher(Bool, f"{self.drone_prefix}/in/load", qos_profile)
        self.__hold_pub   = node.create_publisher(Bool, f"{self.drone_prefix}/in/hold", qos_profile)
        self.__drop_pub   = node.create_publisher(Bool, f"{self.drone_prefix}/in/drop", qos_profile)
        self.__track_pub  = node.create_publisher(Bool, f"{self.drone_prefix}/in/track", qos_profile)

    ### Properties ###
    @property
    def received_arm_ready_signal(self) -> UInt32:
        return self.__arm_ready_signal
    
    @property
    def received_loaded_signal(self) -> bool:
        return self.__loaded_signal
    
    @property
    def drone_status(self) -> AgentStatus:
        return self.__status_signal
    
    @property
    def drone_state(self) -> int:
        return int(self.__status_signal.state)

    # TODO Track is not implemented
    @property
    def drone_track(self) -> bool:
        return self.__track_signal

    ### Setters ###
    def __set_arm_ready_signal(self, msg : UInt32) -> None:
        self.__arm_ready_signal = msg.data == self.id
    
    def __set_status(self, msg : AgentStatus) -> None:
        self.__status_signal = msg

    def __set_loaded_signal(self, msg : UInt32) -> None:
        self.__loaded_signal = msg.data == self.id

    ### signal ###

    def arm(self):
        self.__arm_pub.publish(Bool({"data" : True}))

    def teleop(self):
        self.__teleop_pub.publish(Bool({"data" : True}))

    def load(self):
        self.__load_pub.publish(Bool({"data" : True}))

    def hold(self):
        self.__hold_pub.publish(Bool({"data" : True}))

    def drop(self):
        self.__drop_pub.publish(Bool({"data" : True}))

    def track(self):
        self.__track_pub.publish(Bool({"data" : True}))
    
    # util function
    def __get_drone_id_msg(self) -> UInt32:
        uint32_msg = UInt32()
        uint32_msg.data = self.id
        return uint32_msg
