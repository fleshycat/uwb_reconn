import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


from px4_msgs.srv import ModeChange, TrajectorySetpoint as TrajectorySetpointSrv, VehicleCommand as VehicleCommandSrv, GlobalPath as GlobalPathSrv
from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor, GlobalPath as GlobalPathMsg

from geometry_msgs.msg import Point
from std_msgs.msg import Empty, UInt8

from enum import Enum
import math
import numpy as np
from mission.px4_cmd import PX4CMD

class ProgressStatus(Enum):
    DISARM=0
    ARM=1
    OFFBOARD=2
    TAKEOFF=3
    MISSION1=4
    MISSION2=5
    MISSION3=6
    MISSION4=7
    MISSION5=8
    Done=9

class StartMission(Node, PX4CMD):
    def __init__(self):
        Node.__init__(self, "start_mission")
        PX4CMD.__init__(self)
        self.initialize_node()
        self.disarmPos=[0,0]
        
    def initialize_node(self):
        self.declare_parameter('system_id_list', [0])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        self.num_drone = self.system_id_list
        
        self.get_logger().info(f"Configure DroneManager {self.system_id_list}")
        
        self.create_Sub_n_Pub()
        
        timer_period_monitoring = 0.5  # seconds
        self.timer_monitoring_ = self.create_timer(timer_period_monitoring, self.in_progress_callback)
        
        self.circle_trajectory = self.generate_circular_trajectory(radius=5, frequency=10, total_time=5)
        self.circleProgressCount = 0
        
        self.currentProgressStatus=ProgressStatus.DISARM
        self.ProgressCheckMask = 0
        self.ProgressCompleteMask = (1 << self.num_drone) - 1
        
        self.REF_LLH = []
        
    def setREF_LLH(self):
        pass
        
    def in_progress_callback(self):
        if not self.monitoring_msg_._pos_x:
            return
        self.get_logger().info(f"Current Progress : {self.currentProgressStatus}")
        
        for sys_id in self.system_id_list:
            if self.currentProgressStatus == ProgressStatus.DISARM:
                self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
                self.disarmPos[0]=self.POSX()
                self.disarmPos[1]=self.POSY()
                
            if self.currentProgressStatus == ProgressStatus.ARM:
                if not self.isArmed():
                    msg = VehicleCommandMsg()
                    msg.target_system = self.system_id_
                    msg.command = 400 # MAV_CMD_COMPONENT_ARM_DISARM=400,
                    msg.param1 = 1.0
                    msg.confirmation=True
                    msg.from_external = True      
                    self.vehicle_command_publisher_.publish(msg)
                else:
                    self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
            
            if self.currentProgressStatus == ProgressStatus.OFFBOARD:
                if not self.isOffboard():
                    msg = VehicleCommandMsg()
                    msg.target_system = self.system_id_
                    msg.command = VehicleCommandMsg.VEHICLE_CMD_DO_SET_MODE
                    msg.param1 = 1.0
                    msg.param2 = 6.0 # PX4_CUSTOM_MAIN_MODE_OFFBOARD=6,
                    msg.from_external = True      
                    self.vehicle_command_publisher_.publish(msg)
                else:
                    self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
            
            if self.currentProgressStatus == ProgressStatus.TAKEOFF:
                setpoint=[self.disarmPos[0], self.disarmPos[1], -1.5]
                success, distance = self.isOnSetpoint(setpoint)
                if not success:
                    self.setpoint(setpoint)
                else:
                    self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
            
            if self.currentProgressStatus == ProgressStatus.MISSION1:
                self.circleProgressCount += 1
                
                setpoint=[self.circle_trajectory[self.circleProgressCount % 1000][0], self.circle_trajectory[self.circleProgressCount % 1000][1] , -1.5]
                self.setpoint(setpoint)

            if self.currentProgressStatus == ProgressStatus.Done:
                self.get_logger().info(f"Current Progress : {self.currentProgressStatus}")
                self.destroy_node()
                rclpy.shutdown()
    
    def generate_circular_trajectory(self, radius=5, frequency=10, total_time=10):
        num_points = int(total_time * frequency)  # 총 샘플 수
        trajectory = []

        for i in range(num_points):
            theta = 2 * np.pi * (i / num_points)  # 각도 (라디안)
            x = radius * np.cos(theta)  # X 좌표
            y = radius * np.sin(theta)  # Y 좌표
            trajectory.append((x, y))
        
        return trajectory
    
def main(args=None):
    rclpy.init(args=args)

    start = StartMission()

    rclpy.spin(start)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    start.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 