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

class Agent(Node):
    def __init__(self, sys_id):
        super().__init__(f"agent_{sys_id}_node")
        self.system_id = sys_id
        self.topic_prefix_fmu = f"/drone{self.system_id}/fmu/"
        
        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        
        self.ocm_msg_ = OffboardControlMode()
        self.ocm_msg_.position = True
        self.ocm_msg_.velocity = False
        self.ocm_msg_.acceleration = False
        self.ocm_msg_.attitude = False
        self.ocm_msg_.body_rate = False
        self.ocm_msg_.actuator = False
        self.get_logger().info(f"{self.topic_prefix_fmu + 'out/monitoring'}")
        self.monitoring_msg_ = Monitoring()
        self.monitoring_subscriber = self.create_subscription(
                                        Monitoring, 
                                        self.topic_prefix_fmu + "out/monitoring", 
                                        self.monitoring_callback, 
                                        qos_profile_sensor_data
                                        )
        
        self.vehicle_command_publisher = self.create_publisher(
                                            VehicleCommandMsg, 
                                            self.topic_prefix_fmu + "in/vehicle_command",
                                            qos_profile_sensor_data
                                            )
        
        self.ocm_publisher = self.create_publisher(
                                OffboardControlMode, 
                                self.topic_prefix_fmu + "in/offboard_control_mode", 
                                qos_profile_sensor_data
                                )
        
        self.traj_setpoint_publisher = self.create_publisher(
                                          TrajectorySetpointMsg, 
                                          self.topic_prefix_fmu + "in/trajectory_setpoint",
                                          qos_profile_sensor_data
                                          )
        self.monitoring_flag = False
        
    def timer_ocm_callback(self):
        self.ocm_publisher.publish(self.ocm_msg_)
              
    def monitoring_callback(self, msg):
        self.monitoring_msg_ = msg
        self.monitoring_flag = True
        
    def isArmed(self):
        return self.monitoringFlag(self.monitoring_msg_.status1, MonitoringFlagType.ARM_STATUS.value)
        
    def isOffboard(self):
        return self.monitoringFlag(self.monitoring_msg_.status1, MonitoringFlagType.OFFBOARD_MODE.value)
    
    def isOnSetpoint(self, targetPOS):
        distance = self.distance(self.POS(), targetPOS)
        return (distance < 0.1), distance
    
    def POSX(self):
        return self.monitoring_msg_.pos_x
    
    def POSY(self):
        return self.monitoring_msg_.pos_y
    
    def POSZ(self):
        return self.monitoring_msg_.pos_z
    
    def POS(self):
        return [self.monitoring_msg_.pos_x, 
                self.monitoring_msg_.pos_y, 
                self.monitoring_msg_.pos_z]
    
    def distance(self, pos1, pos2):
        posDiff = [pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]]
        distance = math.sqrt(posDiff[0]**2 + posDiff[1]**2 +  posDiff[2]**2)
        return distance
    
    def setpoint(self, setpoint, yaw=0.0, yawspeed=0.0):
        msg = TrajectorySetpointMsg()
        msg.position[0] = setpoint[0]
        msg.position[1] = setpoint[1]
        msg.position[2] = setpoint[2]
        msg.yaw = yaw
        msg.yawspeed = yawspeed
        self.traj_setpoint_publisher.publish(msg)
            
    def monitoringFlag(self, aValue, aBit):
        return (aValue & (1<<aBit)) > 0