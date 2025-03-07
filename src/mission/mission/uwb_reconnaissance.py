import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import VehicleCommand as VehicleCommandMsg
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, Pose2D

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
    ADJUST      =6
    HOVERING    =7
    RESTART     =8
    COMPLETE    =9

class StartMission(Node):
    def __init__(self):
        super().__init__("Startmission")
        self.tagPos_subscriber = self.create_subscription(Pose2D,
                                                          "tagLLH",
                                                          self.tagPos_callback,
                                                          qos_profile_sensor_data)
        self.initialize_node()
    
    def __del__(self):
        print(f"Node {self.get_name()} is being destroyed.")
        
    def initialize_node(self):
        self.executor = MultiThreadedExecutor()
        
        self.declare_parameter('system_id_list', [0])
        self.declare_parameter('formation_square_length', 1.0)
        self.system_id_list = list(self.get_parameter('system_id_list').get_parameter_value().integer_array_value)
        self.formation_square_length = self.get_parameter('formation_square_length').get_parameter_value().double_value
        
        print(f"Configure DroneManager {self.system_id_list}")
        self.Agent_list = []
        self.takeoff_offset = []
        
        self.init_Agents()
        
        timer_period_monitoring = 0.1  # seconds
        self.timer_progress = self.create_timer(timer_period_monitoring, self.in_progress_callback)
        
        self.currentProgressStatus=ProgressStatus.ARM
        self.ProgressCheckMask = 0
        self.ProgressCompleteMask = (1 << len(self.system_id_list)) - 1
        
        self.detection_message = {}
        self.reArrangeSetpoint = []
        self.adjust_offset = []
        self.HOVERING_count = 0
        self.tagPos = Pose2D()
        self.tagPos_flag = False
        
    def init_Agents(self):
        for sys_id in self.system_id_list:
            self.Agent_list.append(Agent(sys_id))
    
    def tagPos_callback(self, msg):
        self.tagPos = msg
        self.tagPos_flag = True
    
    def in_progress_callback(self):
        if not self.Agent_list[0].monitoring_msg_:
            self.get_logger().info(f"Can not refer to Monitoring msg")
        
        #self.get_logger().info(f"Current Progress : {self.currentProgressStatus}")
        
        if self.currentProgressStatus == ProgressStatus.ARM:
            for drone in self.Agent_list:                
                if not drone.isArmed():
                    #self.get_logger().info(f"Drone {drone.system_id} is not Armed")
                    msg = VehicleCommandMsg()
                    msg.target_system = drone.system_id
                    msg.command = 400 # MAV_CMD_COMPONENT_ARM_DISARM=400,
                    msg.param1 = 1.0
                    msg.confirmation = True
                    msg.from_external = True      
                    drone.vehicle_command_publisher.publish(msg)
                else:
                    #self.get_logger().info(f"Drone {drone.system_id} is Armed")
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
        
        # Search
        # If any drone detects a UWB signal, that drone stops at its current position and switches its state to DETECT
        if self.currentProgressStatus == ProgressStatus.SEARCH:
            detection = False

            for drone in self.Agent_list:
                setpoint=[drone.POSX() + 2, drone.POSY(), -1.5]
                drone.setpoint(setpoint)

                if drone.isDetected():
                    self.get_logger().info(f"Drone {drone.system_id} detected UWB signal.")
                    # Store the Detected Data
                    self.detection_message = {
                        'position': drone.POS(),
                        'sys_id': drone.system_id
                    }
                    detection = True
                    break
            
            if detection:
                self.currentProgressStatus = ProgressStatus.DETECT
        
        # Detect
        # The drone that detected the signal broadcasts a detection message
        # (e.g., detection position and time).
        # In a simulation environment, the message is sent to the GCS, which then broadcasts it to other drones.
        # In a real environment, the message is directly sent to other drones using the J.Fi module.
        # Switch state to REARRANGE.
        if self.currentProgressStatus == ProgressStatus.DETECT:
            self.get_logger().info("Broadcasting detection message to all drones.")
            self.calculate_takeoff_offset()
            detectPos = self.detection_message['position']
            self.reArrangeSetpoint = [detectPos, 
                                 [detectPos[0] + self.formation_square_length, detectPos[1], -1.5],
                                 [detectPos[0], detectPos[1] + self.formation_square_length, -1.5],
                                 [detectPos[0] + self.formation_square_length, detectPos[1] + self.formation_square_length, -1.5],]
            
            n = len(self.system_id_list)
            start = self.detection_message['sys_id'] - 1
            reArrange_adjust = []
            for i in range(n):
                drone = self.Agent_list[(start + i) % n]
                offset = self.takeoff_offset[(start + i) % n]
                sec_offset = self.takeoff_offset[start]
                reArrange_adjust.append([self.reArrangeSetpoint[i][0] + offset[0] - sec_offset[0],
                                         self.reArrangeSetpoint[i][1] + offset[1] - sec_offset[1],])
            self.reArrangeSetpoint = reArrange_adjust
            self.currentProgressStatus = ProgressStatus.REARRANGE
            self.get_logger().info("Rearranging drones.")

        # Rearrange
        # Rearrange the formation so that all drones can detect the UWB signal.
        # Once all drones detect the UWB signal, switch state to HOVERING.
        if self.currentProgressStatus == ProgressStatus.REARRANGE:
            n = len(self.system_id_list)
            start = self.detection_message['sys_id'] - 1
            for i in range(n):
                drone = self.Agent_list[(start + i) % n]
                setpoint = [self.reArrangeSetpoint[i][0], self.reArrangeSetpoint[i][1], -1.5 - 0.2*i]
                success, distance = drone.isOnSetpoint(setpoint)   
                #self.get_logger().info(f"{(start + i) % n + 1} distance : {distance}")         
                if not success:
                    drone.setpoint(setpoint)
                else:
                    self.ProgressCheckMask |= (1 << drone.system_id-1)
            if self.ProgressCheckMask == self.ProgressCompleteMask:
                self.ProgressCheckMask = 0
                self.adjust_offset = self.calculate_adjust_offset()
                self.reArrangeSetpoint = [[setpoint[0] - self.adjust_offset[1],
                                           setpoint[1] - self.adjust_offset[0],]
                                          for setpoint in self.reArrangeSetpoint]
                self.currentProgressStatus = ProgressStatus(self.currentProgressStatus.value + 1)

        if self.currentProgressStatus == ProgressStatus.ADJUST:
            if self.tagPos_flag == False:
                print("Can't Localize the tag")
                return
            self.get_logger().info(f"####progress####")
            self.get_logger().info(f"adjust_offset : {self.adjust_offset}")
            self.get_logger().info(f"reArrangeSetpoint : {self.reArrangeSetpoint}")
            n = len(self.system_id_list)
            start = self.detection_message['sys_id'] - 1
            for i in range(n):
                drone = self.Agent_list[(start + i) % n]
                # setpoint = [self.reArrangeSetpoint[i][0] - self.adjust_offset[1], self.reArrangeSetpoint[i][1] - self.adjust_offset[0], -1.5]
                setpoint = self.reArrangeSetpoint[i] + [-1.5]
                success, distance = drone.isOnSetpoint(setpoint)   
                #self.get_logger().info(f"{(start + i) % n + 1} distance : {distance}")         
                if not success:
                    drone.setpoint(setpoint)
                else:
                    self.ProgressCheckMask |= (1 << drone.system_id-1)
            if self.ProgressCheckMask == self.ProgressCompleteMask:
                self.ProgressCheckMask = 0
                self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
                self.get_logger().info("Hovering drones.")
            
        # Hovering
        # Calculate the precise node position. 
        # Switch state to COMPLETE.
        if self.currentProgressStatus == ProgressStatus.HOVERING:
            self.HOVERING_count += 1
            
            n = len(self.system_id_list)
            start = self.detection_message['sys_id'] - 1
            
            for i in range(n):
                drone = self.Agent_list[(start + i) % n]
                setpoint = self.reArrangeSetpoint[i] + [-1.5]
                success, distance = drone.isOnSetpoint(setpoint)             
                drone.setpoint(setpoint)
            
            if self.HOVERING_count > 100:
                self.currentProgressStatus = ProgressStatus.COMPLETE
                self.HOVERING_count = 0

        if self.currentProgressStatus == ProgressStatus.COMPLETE:
            print(f"Current Progress : {self.currentProgressStatus}")
            self.destroy_node()
            rclpy.shutdown()
    
    def calculate_takeoff_offset(self):
        self.takeoff_offset.clear()
        ref_LLH = [self.Agent_list[0].monitoring_msg_.ref_lat, self.Agent_list[0].monitoring_msg_.ref_lon, self.Agent_list[0].monitoring_msg_.ref_alt]
        for drone in self.Agent_list:
            LLH = [drone.monitoring_msg_.ref_lat, drone.monitoring_msg_.ref_lon, drone.monitoring_msg_.ref_alt]
            NED = LLH2NED(LLH, ref_LLH)
            offset = [-i for i in NED]
            self.get_logger().info(f"drone {drone.system_id} offset : {offset}")
            self.takeoff_offset.append(offset)
    
    def calculate_adjust_offset(self):
        dronePose = [drone.POS() for drone in self.Agent_list]
        dronePose = [[dronePose[i][0] - self.takeoff_offset[i][0], 
                      dronePose[i][1] - self.takeoff_offset[i][1],] 
                     for i in range(len(self.system_id_list))]
        center = np.mean(dronePose, axis=0)
        ref_LLH = [self.Agent_list[0].monitoring_msg_.ref_lat, self.Agent_list[0].monitoring_msg_.ref_lon, self.Agent_list[0].monitoring_msg_.ref_alt]
        tagpos = [self.tagPos.x, self.tagPos.y, 0.0]
        tagPOS_NED = LLH2NED(tagpos, ref_LLH)
        adjust_offset = [tagPOS_NED[0] - center[0], tagPOS_NED[1]- center[1]]
        self.get_logger().info(f"center:{center}")
        self.get_logger().info(f"tagNED:{tagPOS_NED}")
        self.get_logger().info(f"adjust_offset : {adjust_offset}")
        return adjust_offset
    
# WGS-84 
a = 6378137.0 
f = 1.0 / 298.257223563  
e2 = 2 * f - f * f 
 
def NED2LLH(NED, ref_LLH):
    lat_ref = np.deg2rad(ref_LLH[0])
    lon_ref = np.deg2rad(ref_LLH[1])
    
    sin_lat_ref = np.sin(lat_ref)
    cos_lat_ref = np.cos(lat_ref)
    
    N_ref = a / np.sqrt(1 - e2 * sin_lat_ref**2)

    dlat = NED[0] / N_ref
    dlon = NED[1] / (N_ref * cos_lat_ref)

    # 결과 LLH 계산
    lat = lat_ref + dlat
    lon = lon_ref + dlon
    h = ref_LLH[2] + NED[2]
    
    return [np.rad2deg(lat), np.rad2deg(lon), h]

def LLH2NED(LLH, ref_LLH):
    lat_ref = np.deg2rad(ref_LLH[0])
    lon_ref = np.deg2rad(ref_LLH[1])
    lat = np.deg2rad(LLH[0])
    lon = np.deg2rad(LLH[1])
    
    sin_lat_ref = np.sin(lat_ref)
    cos_lat_ref = np.cos(lat_ref)
    
    N_ref = a / np.sqrt(1 - e2 * sin_lat_ref**2)

    dlat = lat - lat_ref
    dlon = lon - lon_ref

    NED_N = dlat * N_ref
    NED_E = dlon * N_ref * cos_lat_ref
    NED_D = LLH[2] - ref_LLH[2]
    
    return [NED_N, NED_E, NED_D]
    
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
    # start.destroy_node()
    for i in start.Agent_list:
        i.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main() 