import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import Monitoring, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommand as VehicleCommandMsg

from geometry_msgs.msg import Point
from std_msgs.msg import Empty, UInt8

from enum import Enum
import math
import numpy as np

class MonitoringFlagType(Enum):
    SAFETY_LOCK_STATUS = 0
    ARM_STATUS = 1
    OFFBOARD_MODE = 2
    MANUAL_MODE = 3
    AUTO_MODE = 4 
    FAIL_SAFE_MODE = 5           # Not used

class PX4CMD(Node):
    def __init__(self):
        super().__init__("ocm")
        
        self.declare_parameter('system_id', 1)
        self.system_id_ = self.get_parameter('system_id').get_parameter_value().integer_value
        self.topic_prefix_fmu_ = f"drone{self.system_id_}/fmu/"
        
        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        
        self.ocm_publisher_ = self.create_publisher(OffboardControlMode, f'{self.topic_prefix_fmu_}in/offboard_control_mode', qos_profile_sensor_data)
        self.ocm_msg_ = OffboardControlMode()
        self.ocm_msg_.position = True
        self.ocm_msg_.velocity = False
        self.ocm_msg_.acceleration = False
        self.ocm_msg_.attitude = False
        self.ocm_msg_.body_rate = False
        self.ocm_msg_.actuator = False
        
    def timer_ocm_callback(self):
        self.ocm_publisher_.publish(self.ocm_msg_)
        
        
def main(args=None):
    rclpy.init(args=args)

    start = PX4CMD()

    rclpy.spin(start)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    start.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 