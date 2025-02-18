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

class PX4CMD():
    def __init__(self):
        self.system_id_list = [ 1 ]
        self.topic_prefix_fmu_ = ""
        
        self.monitoring_msg_ = Monitoring()
        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        
        self.ocm_msg_ = OffboardControlMode()
        self.ocm_msg_.position = True
        self.ocm_msg_.velocity = False
        self.ocm_msg_.acceleration = False
        self.ocm_msg_.attitude = False
        self.ocm_msg_.body_rate = False
        self.ocm_msg_.actuator = False
        
        self.monitoring_flag = False
        
        self.REF_Pos = np.array()
        
    def create_Sub_n_Pub(self):
        topic_prefix_fmu_list = [f"drone{sys_id}/fmu/" for sys_id in self.system_id_list]
        self.monitoring_subscribers = [ self.create_subscription(
                                        Monitoring, 
                                        f'{topic_prefix_fmu}out/monitoring', 
                                        self.monitoring_callback, 
                                        qos_profile_sensor_data
                                        ) for topic_prefix_fmu in topic_prefix_fmu_list
                                       ]
        self.vehicle_command_publishers = [ self.create_publisher(
                                            VehicleCommandMsg, 
                                            f'{topic_prefix_fmu}in/vehicle_command', 
                                            qos_profile_sensor_data
                                            ) for topic_prefix_fmu in topic_prefix_fmu_list
                                           ] 
        self.ocm_publishers = [ self.create_publisher(
                                OffboardControlMode, 
                                f'{topic_prefix_fmu}in/offboard_control_mode', 
                                qos_profile_sensor_data
                                ) for topic_prefix_fmu in topic_prefix_fmu_list
                               ]
        self.traj_setpoint_publishers = [ self.create_publisher(
                                          TrajectorySetpointMsg, 
                                          f'{topic_prefix_fmu}in/trajectory_setpoint', 
                                          qos_profile_sensor_data
                                          ) for topic_prefix_fmu in topic_prefix_fmu_list
                                         ]
        
    def timer_ocm_callback(self):
        self.ocm_publisher_.publish(self.ocm_msg_)
              
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
        self.traj_setpoint_publisher_.publish(msg)
            
    def monitoringFlag(self, aValue, aBit):
        return (aValue & (1<<aBit)) > 0
    
    def initREF_Pose(self):
        pass