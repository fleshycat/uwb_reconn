import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


from px4_msgs.srv import ModeChange, TrajectorySetpoint as TrajectorySetpointSrv, VehicleCommand as VehicleCommandSrv, GlobalPath as GlobalPathSrv
from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor, GlobalPath as GlobalPathMsg

from geometry_msgs.msg import Point
from std_msgs.msg import Empty, UInt8

from enum import Enum
import math

class MonitoringFlagType(Enum):
        SAFETY_LOCK_STATUS = 0
        ARM_STATUS = 1
        OFFBOARD_MODE = 2
        MANUAL_MODE = 3
        AUTO_MODE = 4 
        FAIL_SAFE_MODE = 5           # Not used
        
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

class StartMission(Node):
    def __init__(self):
        super().__init__("start_mission")
        self.initialize_node()
        self.disarmPos=[0,0]
        
    def initialize_node(self):
        self.declare_parameter('system_id', 1)
        self.system_id_ = self.get_parameter('system_id').get_parameter_value().integer_value
        
        self.declare_parameter('robot_type', 'iris')
        self.robot_type_ = self.get_parameter('robot_type').get_parameter_value().string_value
        
        self.get_logger().info(f"Configure DroneManager {self.system_id_}")
        
        self.topic_prefix_fmu_ = f"drone{self.system_id_}/fmu/"
        
        self.monitoring_subscriber_ = self.create_subscription(Monitoring, f'{self.topic_prefix_fmu_}out/monitoring', self.monitoring_callback, qos_profile_sensor_data)
        self.monitoring_msg_ = Monitoring()
        
        self.traj_setpoint_publisher_ = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_fmu_}in/trajectory_setpoint', qos_profile_sensor_data)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommandMsg, f'{self.topic_prefix_fmu_}in/vehicle_command', qos_profile_sensor_data)
        
        timer_period_monitoring = 0.5  # seconds
        self.timer_monitoring_ = self.create_timer(timer_period_monitoring, self.in_progress_callback)
        
        timer_period_ocm = 0.1
        self.timer_ocm_ = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        
        self.currentProgressStatus=ProgressStatus.DISARM
        
        self.ocm_publisher_ = self.create_publisher(OffboardControlMode, f'{self.topic_prefix_fmu_}in/offboard_control_mode', qos_profile_sensor_data)
        self.ocm_msg_ = OffboardControlMode()
        self.ocm_msg_.position = True
        self.ocm_msg_.velocity = False
        self.ocm_msg_.acceleration = False
        self.ocm_msg_.attitude = False
        self.ocm_msg_.body_rate = False
        self.ocm_msg_.actuator = False
        
        self.monitoring_flag = False
        
        self.mission_time_count = 0
        
    def timer_ocm_callback(self):
        self.ocm_publisher_.publish(self.ocm_msg_)
        
    def in_progress_callback(self):
        if not self.monitoring_msg_._pos_x:
            return
        print("Current Progress :", self.currentProgressStatus)
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
            if self.robot_type_ == "rover":
                self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 1)
            else:
                setpoint=[self.disarmPos[0], self.disarmPos[1], -1.5]
                success, distance = self.isOnSetpoint(setpoint)
                if not success:
                    self.setpoint(setpoint)
                    print("distance", distance)
                    print(f"{self.robot_type_} : {setpoint}")
                else:
                    if self.mission_time_count < 60:
                        self.mission_time_count += 1
                    else:
                        self.currentProgressStatus=ProgressStatus(self.currentProgressStatus.value + 6)
                        self.disarmPos=[self.POSX(), self.POSY()]        
        
        if self.currentProgressStatus == ProgressStatus.Done:
            print("Current Progress :", self.currentProgressStatus)
            self.destroy_node()
            rclpy.shutdown()
              
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
    
    def setpoint(self, setpoint, yaw=0.0):
        msg = TrajectorySetpointMsg()
        msg.position[0] = setpoint[0]
        msg.position[1] = setpoint[1]
        msg.position[2] = setpoint[2]
        msg.yaw = yaw
        msg.yawspeed = 0.0
        self.traj_setpoint_publisher_.publish(msg)
            
    def monitoringFlag(self, aValue, aBit):
        return (aValue & (1<<aBit)) > 0
    
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