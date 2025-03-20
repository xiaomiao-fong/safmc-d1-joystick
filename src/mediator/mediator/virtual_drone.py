from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from esp_msg.msg import ESPCMD, AgentStatus
from std_msgs.msg import Bool, UInt32
from espkinesis_msgs.msg import ChannelOverride
from safmc_msgs.msg import Magnet

class Drone():

    def __init__(self, id: int, node: Node):
        # Init some value
        self.id = id
        self.mediator_node = node
        self.drone_prefix = f"/drone_{self.id}"
        self.px4_prefix = f"/px4_{self.id}"
        self.target_prefix = f"/target_{self.id}"

        self.drop_cnt = 0

        self.__arm_ready_signal: bool = False
        self.__status_signal: AgentStatus = None
        self.__loaded_signal: bool = False

        self._is_armed = False
        self._is_each_pre_flight_check_passed = False
        self._vehicle_timestamp = 1

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_target = QoSProfile(
            depth=10
        )

        # ESP Kinesis

        self.__target_publisher = node.create_publisher(
            ChannelOverride,
            f'{self.target_prefix}/espk/channel_override',
            qos_target
        )

        # # Subscriber for px4

        self.vehicle_status_sub = node.create_subscription(
            VehicleStatus,
            f"{self.px4_prefix}/fmu/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )

        # Subscriber for drone
        self.arm_ready_sub = node.create_subscription(
            UInt32,
            f"{self.px4_prefix}/out/arm_ready",
            self.__set_arm_ready_signal,
            qos_profile
        )

        self.status_sub = node.create_subscription(
            AgentStatus,
            f"{self.px4_prefix}/out/status",
            self.__set_status,
            qos_profile
        )

        self.loaded_sub = node.create_subscription(
            UInt32,
            f"{self.px4_prefix}/out/loaded",
            self.__set_loaded_signal,
            qos_profile
        )

        # Publishers for drone
        self.__idle_pub   = node.create_publisher(Bool, f"{self.px4_prefix}/in/idle", qos_profile)
        self.__arm_pub    = node.create_publisher(Bool, f"{self.px4_prefix}/in/arm", qos_profile)
        self.__teleop_pub = node.create_publisher(Bool, f"{self.px4_prefix}/in/teleop", qos_profile)
        self.__magnet_pub   = node.create_publisher(Magnet, f"{self.px4_prefix}/payload/in/magnet", 10)
        self.__hold_pub   = node.create_publisher(Bool, f"{self.px4_prefix}/in/hold", qos_profile)
        self.__drop_pub   = node.create_publisher(Bool, f"{self.px4_prefix}/in/drop", qos_profile)
        self.__track_pub  = node.create_publisher(Bool, f"{self.px4_prefix}/in/track", qos_profile)

    ### Properties ###
    @property
    def received_arm_ready_signal(self) -> UInt32:
        return self.__arm_ready_signal
    
    @property
    def received_loaded_signal(self) -> UInt32:
        return self.__loaded_signal
    
    @property
    def drone_status(self) -> AgentStatus:
        return self.__status_signal
    
    @property
    def drone_state(self) -> int:
        if self.__status_signal is None: return 8
        return int(self.__status_signal.state)
    
    @property
    def is_armed(self) -> bool:
        return self._is_armed
    
    @property
    def is_pf_ok(self) -> bool:
        return self._is_each_pre_flight_check_passed

    ### Setters ###

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self._is_each_pre_flight_check_passed = (
            vehicle_status_msg.pre_flight_checks_pass
        )
        self._vehicle_timestamp = vehicle_status_msg.timestamp
        self._is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def __set_arm_ready_signal(self, msg : UInt32) -> None:
        self.__arm_ready_signal = msg.data == self.id
    
    def __set_status(self, msg : AgentStatus) -> None:
        self.__status_signal = msg

    def __set_loaded_signal(self, msg : UInt32) -> None:
        self.__loaded_signal = msg.data == self.id

    ### signal ###

    def arm(self):

        print(self.id, "arm")
        msg = ChannelOverride()

        msg.channels = [-1,-1,-1,-1,2000]
        msg.duration = 1000

        msg.bypass_safety = True
        self.__target_publisher.publish(msg)

    def teleop(self, velocities):
        msg = ChannelOverride()

        print(velocities)
        msg.channels = [int(velocities[0]),int(velocities[1]),int(velocities[2]),int(velocities[3])]
        msg.duration = 1000

        self.__target_publisher.publish(msg)

    def load(self):
        msg = Magnet()
        self.drop_cnt = 0
        msg.magnet1 = True
        msg.magnet2 = True
        msg.magnet3 = True

        self.__magnet_pub.publish(msg)

    def hold(self):
        self.__hold_pub.publish(Bool(**{"data" : True}))

    def drop(self):
        msg = Magnet()
        msg.magnet1 = self.drop_cnt < 0
        msg.magnet2 = self.drop_cnt < 1
        msg.magnet3 = self.drop_cnt < 2

        self.drop_cnt += 1
        self.drop_cnt %= 3

        self.__magnet_pub.publish(msg)

    def track(self):
        self.__track_pub.publish(Bool(**{"data" : True}))

    def idle(self):
        print(self.id, "idle")
        self.__idle_pub.publish(Bool(**{"data" : True}))
    
    # util function
    def __get_drone_id_msg(self) -> UInt32:
        uint32_msg = UInt32()
        uint32_msg.data = self.id
        return uint32_msg