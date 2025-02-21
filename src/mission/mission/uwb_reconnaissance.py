import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import VehicleCommand as VehicleCommandMsg
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point

from enum import Enum
import math
import numpy as np
from mission.Agent import Agent

class ProgressStatus(Enum):
    ARM         =0
    OFFBOARD    =1
    TAKEOFF     =2
    SEARCH      =3
    DETECT      =4
    REARRANGE   =5
    HOVERING    =6
    RESTART     =7
    COMPLETE        =8

class StartMission(Node):
    def __init__(self):
        super().__init__("Startmission")
        self.initialize_node()
        
    def initialize_node(self):
        self.executor = MultiThreadedExecutor()
        
        self.declare_parameter('system_id_list', [0])
        self.system_id_list = list(self.get_parameter('system_id_list').get_parameter_value().integer_array_value)
        
        print(f"Configure DroneManager {self.system_id_list}")
        self.Agent_list = []
        
        self.init_Agents()
        
        timer_period_monitoring = 0.1  # seconds
        self.timer_progress = self.create_timer(timer_period_monitoring, self.in_progress_callback)
        
        self.circle_trajectory = self.generate_circular_trajectory(radius=5, frequency=10, total_time=5)
        self.circleProgressCount = 0
        
        self.currentProgressStatus=ProgressStatus.ARM
        self.ProgressCheckMask = 0
        self.ProgressCompleteMask = (1 << len(self.system_id_list)) - 1
        
    def init_Agents(self):
        for sys_id in self.system_id_list:
            self.Agent_list.append(Agent(sys_id))
    
    def in_progress_callback(self):
        if not self.Agent_list[0].monitoring_msg_:
            self.get_logger().info(f"Can not refer to Monitoring msg")
        
        self.get_logger().info(f"Current Progress : {self.currentProgressStatus}")
        
        if self.currentProgressStatus == ProgressStatus.ARM:
            for drone in self.Agent_list:                
                if not drone.isArmed():
                    self.get_logger().info(f"Drone {drone.system_id} is not Armed")
                    msg = VehicleCommandMsg()
                    msg.target_system = drone.system_id
                    msg.command = 400 # MAV_CMD_COMPONENT_ARM_DISARM=400,
                    msg.param1 = 1.0
                    msg.confirmation = True
                    msg.from_external = True      
                    drone.vehicle_command_publisher.publish(msg)
                else:
                    self.get_logger().info(f"Drone {drone.system_id} is Armed")
                    self.ProgressCheckMask |= (1 << drone.system_id-1)
                    
            if self.ProgressCheckMask == self.ProgressCompleteMask:
                self.ProgressCheckMask = 0
                self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
            
        if self.currentProgressStatus == ProgressStatus.OFFBOARD:
            for drone in self.Agent_list:                
                if not drone.isOffboard():
                    msg = VehicleCommandMsg()
                    msg.target_system = drone.system_id
                    msg.command = VehicleCommandMsg.VEHICLE_CMD_DO_SET_MODE
                    msg.param1 = 1.0
                    msg.param2 = 6.0 # PX4_CUSTOM_MAIN_MODE_OFFBOARD=6,
                    msg.from_external = True      
                    drone.vehicle_command_publisher.publish(msg)
                else:
                    self.ProgressCheckMask |= (1 << drone.system_id-1)
            if self.ProgressCheckMask == self.ProgressCompleteMask:
                self.ProgressCheckMask = 0
                self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
        
        if self.currentProgressStatus == ProgressStatus.TAKEOFF:
            for drone in self.Agent_list:
                setpoint=[drone.POSX(), drone.POSY(), -1.5]
                success, distance = drone.isOnSetpoint(setpoint)            
                if not success:
                    drone.setpoint(setpoint) 
                else:
                    self.ProgressCheckMask |= (1 << drone.system_id-1)
                    
            if self.ProgressCheckMask == self.ProgressCompleteMask:
                self.ProgressCheckMask = 0
                self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
            
        if self.currentProgressStatus == ProgressStatus.SEARCH:
            self.circleProgressCount += 1
            for drone in self.Agent_list:
                setpoint=[self.circle_trajectory[self.circleProgressCount % 50][0], self.circle_trajectory[self.circleProgressCount % 50][1] , -1.5]
                drone.setpoint(setpoint)

        if self.currentProgressStatus == ProgressStatus.COMPLETE:
            print(f"Current Progress : {self.currentProgressStatus}")
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
    
    executor = MultiThreadedExecutor()
    executor.add_node(start)
    for i in start.Agent_list:
        executor.add_node(i)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    start.destroy_node()
    for i in start.Agent_list:
        i.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 