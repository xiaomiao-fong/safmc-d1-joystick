import rclpy
from .coordinate import Coordinate
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from esp_msg.msg import ESPCMD, State, AgentStatus
from std_msgs.msg import Bool, UInt32
from mediator.constants import NUM_DRONES, NUM_BUTTONS

class Mediator(Node):

    def __init__(self):
        super().__init__("Mediator")
        self.get_logger().info("Start mediating")
        NUM_DRONES = 3

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.espcmd = ESPCMD()
        self.arm_ready = [False] * (NUM_DRONES + 1)
        self.load_ready = [False] * (NUM_DRONES + 1)

        # Subscriber
        self.__topic_prefix = f"/drone_1"

        self.arm_pub = self.create_publisher(Bool, f"{self.__topic_prefix}/arm", qos_profile)
        self.teleop_pub = self.create_publisher(Bool, f"{self.__topic_prefix}/teleop", qos_profile)
        self.load_pub = self.create_publisher(Bool, f"{self.__topic_prefix}/load", qos_profile)

        # TODO mediator outgoing msg
        self.drop_pub = self.create_publisher(Bool, f"{self.__topic_prefix}/drop", qos_profile)
        self.create_publisher(Bool, f"{self.__topic_prefix}/track", qos_profile)

        # Publisher

        self.create_subscription(ESPCMD, '/esp_values', self.__espcmd, qos_profile)
        self.arm_ready_sub = self.create_subscription(UInt32, f"/mediator/arm_ready", self.__arm_ready, qos_profile)
        # self.status_sub = self.create_subscription(AgentStatus, '/mediator/status', qos_profile)
        self.loaded_sub = self.create_subscription(UInt32, '/mediator/loaded', self.__load_ready, qos_profile)

    
    #

    def __espcmd(self, msg):
        print((msg.buttons[0]^self.espcmd.buttons[0]), (self.arm_ready[1] or self.load_ready[1]))
        if (msg.buttons[0]^self.espcmd.buttons[0]) and (self.arm_ready[1] or self.load_ready[1]):
            print("test")
            teleop_msg = Bool()
            teleop_msg.data = True
            self.teleop_pub.publish(teleop_msg)
        
        if (msg.buttons[1]^self.espcmd.buttons[1]):
            load_msg = Bool()
            load_msg.data = True
            self.load_pub.publish(load_msg)
        
        if (msg.buttons[2]^self.espcmd.buttons[2]):
            drop_msg = Bool()
            drop_msg.data = True
            self.drop_pub.publish(drop_msg)
        
        self.espcmd = msg

    def __arm_ready(self, msg : UInt32):
        self.get_logger().info(f"ARM READY {msg.data}")
        self.arm_ready[msg.data] = True
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

    def __load_ready(self, msg : UInt32):
        self.get_logger().info("LOAD READY")
        self.load_ready[msg.data] = True
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        
def main():
    rclpy.init()
    mediator = Mediator()
    rclpy.spin(mediator)
    rclpy.shutdown()

if __name__ == "__main__":
    main()






























































