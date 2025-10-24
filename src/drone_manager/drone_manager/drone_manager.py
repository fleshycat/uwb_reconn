import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from px4_msgs.srv import ModeChange, TrajectorySetpoint as TrajectorySetpointSrv, VehicleCommand as VehicleCommandSrv, GlobalPath as GlobalPathSrv
from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor, GlobalPath as GlobalPathMsg
from uwb_msgs.msg import RangingDiff, Ranging

## only for simulation ##
import std_msgs.msg as std_msgs
from sensor_msgs.msg  import PointCloud2
from sensor_msgs_py import point_cloud2

from std_msgs.msg import Header, UInt8

from drone_manager.class_particle import ParticleFilter
from drone_manager.formation import FormationForce
from drone_manager.repulsion import RepulsionForce
from drone_manager.target import TargetForce
from drone_manager.mode_handler import ModeHandler, Mode

import math
import numpy as np

class DroneManager(Node):
    def __init__(self):
        super().__init__("drone_manager")
        self.drone_manager_delcare_parameters()
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        self.formation_side_length = self.get_parameter('formation_side_length').get_parameter_value().double_value
        self.mission_zlevel = self.get_parameter('mission_zlevel').get_parameter_value().double_value

        self.get_logger().info(f"Configure DroneManager {self.system_id}")

        self.topic_prefix_manager = f"drone{self.system_id}/manager/"  #"drone1/manager/"
        self.topic_prefix_fmu = f"drone{self.system_id}/fmu/"          #"drone1/fmu/"
        self.topic_prefix_uwb = f"drone{self.system_id}/uwb/ranging"
        
        self.monitoring_msg = Monitoring()
        self.uwb_sub_msg = RangingDiff()
        self.global_path = []
        self.global_path_threshold = 0.1
        self.takeoff_offset_dic = {}
        self.agent_uwb_range_dic = {f'{i}':Ranging() for i in self.system_id_list}
        self.agent_target_dic = {}
        
        self.direction = TrajectorySetpointMsg()

        ## Publisher ##
        self.ocm_publisher = self.create_publisher(OffboardControlMode, f'{self.topic_prefix_fmu}in/offboard_control_mode', qos_profile_sensor_data)                    #"drone1/fmu/in/offboard_control_mode"
        self.uwb_ranging_publisher = self.create_publisher(Ranging, f'{self.topic_prefix_manager}out/ranging', qos_profile_sensor_data)
        self.traj_setpoint_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_fmu}in/trajectory_setpoint', qos_profile_sensor_data)
        self.target_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_manager}out/target', qos_profile_sensor_data)
        self.particle_publisher = self.create_publisher(PointCloud2, f'{self.topic_prefix_manager}out/particle_cloud', qos_profile_sensor_data)
        self.monitoring_publisher = self.create_publisher(Monitoring, f'{self.topic_prefix_manager}out/monitoring', qos_profile_sensor_data)
        self.total_gradient_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_manager}out/gradient', qos_profile_sensor_data)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommandMsg, f'{self.topic_prefix_fmu}in/vehicle_command', qos_profile_sensor_data)

        ## Subscriber ##
        self.uwb_subscriber = self.create_subscription(RangingDiff, self.topic_prefix_uwb, self.uwb_msg_callback, qos_profile_sensor_data)
        self.monitoring_subscriber = self.create_subscription(Monitoring, f'{self.topic_prefix_fmu}out/monitoring', self.monitoring_callback, qos_profile_sensor_data)  #"drone1/fmu/out/monitoring"
        self.timestamp_subscriber = self.create_subscription(Header, f'qhac/manager/in/timestamp',self.timestamp_callback, 10)
        self.global_path_subscriber = self.create_subscription(GlobalPathMsg, f'{self.topic_prefix_manager}in/global_path', self.global_path_callback, 10)
        self.mode_change_subscriber = self.create_subscription(UInt8, f'{self.topic_prefix_manager}in/mode_change', self.mode_change_callback, 10)
        self.agent_uwb_range_subscribers = [
            self.create_subscription(Ranging, f'drone{i}/manager/out/ranging', self.make_uwb_range_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list if i != self.system_id
        ]
        self.agent_target_subscribers = [
            self.create_subscription(TrajectorySetpointMsg, f'drone{i}/manager/out/target', self.make_target_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list if i != self.system_id
        ]
        self.add_on_set_parameters_callback(self.set_parameters_callback)
        
        ## OCM Msg ##
        self.ocm_msg = OffboardControlMode()
        
        ## Potential Field ##
        self.desired_formation = self.get_desired_formation(self.formation_side_length)
        self.f_formation = FormationForce(desired_positions = self.desired_formation,
                                        k_scale=    self.get_parameter("formation_k_scale").get_parameter_value().double_value,
                                        k_pair=     self.get_parameter("formation_k_pair").get_parameter_value().double_value,
                                        k_shape=    self.get_parameter("formation_k_shape").get_parameter_value().double_value,
                                        k_z=        self.get_parameter("formation_k_z").get_parameter_value().double_value)
        self.tol = self.get_parameter("formation_tolerance").get_parameter_value().double_value
        self.f_repulsion = RepulsionForce(n_agents=len(self.system_id_list),
                                        c_rep=  self.get_parameter("repulsion_c_rep").get_parameter_value().double_value,
                                        cutoff= self.get_parameter("repulsion_cutoff").get_parameter_value().double_value,
                                        sigma=  self.get_parameter("repulsion_sigma").get_parameter_value().double_value)
        self.f_target = TargetForce([0,0], k_target= self.get_parameter("target_k").get_parameter_value().double_value)
        self.target_bound = self.formation_side_length / (2.0 * math.sin(math.pi / len(self.system_id_list)))
        weight_table = self.get_parameter("weight_table").get_parameter_value().integer_array_value
        self.weight_table = [(weight_table[i], weight_table[i+1], weight_table[i+2]) for i in range(0, len(weight_table), 3)]
        self.desired_yaw = 0.0
        self.return_hold_time = self.get_parameter("return_hold_time").get_parameter_value().double_value
        
        ## Particle Filter ##
        self.num_particles = self.get_parameter("num_particles").get_parameter_value().integer_value
        self.particle_filter = ParticleFilter(num_particles=self.num_particles)
        self.target = []
        self.have_target = False
        self.uwb_data_list = []
        self.uwb_threshold = self.get_parameter("uwb_threshold").get_parameter_value().double_value

        ## ModeHandler ##
        self.mode_handler = ModeHandler(self)
        self.handle_flag = False
        self.collection_step = 0
        
        ## Timer ##
        self.timer_start()
        
        self.gcs_timestamp = Header()
        self.init_timestamp = self.get_clock().now().to_msg().sec
        
        self.initiate_drone_manager()
    
    def drone_manager_delcare_parameters(self):
        # Declare parameters for the drone manager
        self.declare_parameter('system_id', 1)
        self.declare_parameter('system_id_list', [1,2,3,4])
        self.declare_parameter('formation_side_length', 6.0)
        self.declare_parameter('mission_zlevel', 5.0)
        # Formation parameters
        self.declare_parameter("formation_k_scale", 0.0)
        self.declare_parameter("formation_k_pair", 4.0)
        self.declare_parameter("formation_k_shape", 4.0)
        self.declare_parameter("formation_k_z", 4.0)
        self.declare_parameter("formation_tolerance", 1.5)
        self.declare_parameter("return_hold_time", 10.0)
        # Repulsion and target parameters
        self.declare_parameter("repulsion_c_rep", 5.0)  # Repulsion constant
        self.declare_parameter("repulsion_cutoff", 4.0)  # Cutoff distance for repulsion
        self.declare_parameter("repulsion_sigma", 5.0)  # Sigma for repulsion force
        # Target parameters
        self.declare_parameter("target_k", 2.0)  # Target force constant
        self.declare_parameter("weight_table", [10,1,1, 10,1,0, 10,1,2])  # Weights for repulsion, target, and formation
        # Particle filter parameters
        self.declare_parameter("num_particles", 700)  # Number of particles for the particle filter
        self.declare_parameter("uwb_threshold", 10.0)  # Threshold for UWB ranging in meters
        # Timer parameters
        self.declare_parameter("timer_ocm_period", 0.1)  # Timer period for OCM in seconds 10hz
        self.declare_parameter("timer_uwb_period", 0.1)  # Timer period for UWB in seconds 10hz
        self.declare_parameter("timer_global_path_period", 0.1)  # Timer period for global path in seconds 10hz
        self.declare_parameter("timer_mission_period", 0.05)  # Timer period for mission in seconds 20hz
        self.declare_parameter("timer_monitoring_period", 1.0)  # Timer period for monitoring in seconds 1hz

    def set_parameters_callback(self, params):
        result = SetParametersResult()
        for param in params:
            if param.name == 'formation_side_length':
                self.formation_side_length = param.value
                self.desired_formation = self.get_desired_formation(self.formation_side_length)
                self.f_formation.set_desired_formation(self.desired_formation)
                self.get_logger().info(f"Formation side length set to {self.formation_side_length}")
                self.target_bound = self.formation_side_length / (2.0 * math.sin(math.pi / len(self.system_id_list)))
            elif param.name == 'mission_zlevel':
                self.mission_zlevel = param.value
                self.get_logger().info(f"Mission Z-level set to {self.mission_zlevel}")
            elif param.name == 'formation_k_scale':
                self.f_formation.k_scale = param.value
                self.get_logger().info(f"Formation k_scale set to {self.f_formation.k_scale}")
            elif param.name == 'formation_k_pair':
                self.f_formation.k_pair = param.value
                self.get_logger().info(f"Formation k_pair set to {self.f_formation.k_pair}")
            elif param.name == 'formation_k_shape':
                self.f_formation.k_shape = param.value
                self.get_logger().info(f"Formation k_shape set to {self.f_formation.k_shape}")
            elif param.name == 'formation_k_z':
                self.f_formation.k_z = param.value
                self.get_logger().info(f"Formation k_z set to {self.f_formation.k_z}")
            elif param.name == 'formation_tolerance':   
                self.tol = param.value
                self.get_logger().info(f"Formation tolerance set to {self.tol}")
            elif param.name == 'repulsion_c_rep':
                self.f_repulsion.c_rep = param.value
                self.get_logger().info(f"Repulsion c_rep set to {self.f_repulsion.c_rep}")
            elif param.name == 'repulsion_cutoff':
                self.f_repulsion.cutoff = param.value
                self.get_logger().info(f"Repulsion cutoff set to {self.f_repulsion.cutoff}")
            elif param.name == 'repulsion_sigma':
                self.f_repulsion.sigma = param.value
                self.get_logger().info(f"Repulsion sigma set to {self.f_repulsion.sigma}")
            elif param.name == 'target_k':
                self.f_target.k_target = param.value
                self.get_logger().info(f"Target k set to {self.f_target.k_target}")
            elif param.name == 'num_particles':
                self.particle_filter.set_num_particles(param.value)
                self.get_logger().info(f"Number of particles set to {self.particle_filter.num_particles}")
            elif param.name == 'uwb_threshold':
                self.uwb_threshold = param.value
                self.get_logger().info(f"UWB threshold set to {self.uwb_threshold} meters")
            elif param.name == 'weight_table':
                weight_table = param.value
                self.weight_table = [(weight_table[i], weight_table[i+1], weight_table[i+2]) for i in range(0, len(weight_table), 3)]
                self.get_logger().info(f"Weight table updated: {self.weight_table}")
            elif param.name == 'timer_ocm_period':
                self.timer_ocm.cancel()
                self.timer_ocm = self.create_timer(param.value, self.timer_ocm_callback)
                self.get_logger().info(f"OCM timer period set to {param.value} seconds")
            elif param.name == 'timer_uwb_period':
                self.timer_uwb.cancel()
                self.timer_uwb = self.create_timer(param.value, self.timer_uwb_callback)
                self.get_logger().info(f"UWB timer period set to {param.value} seconds")
            elif param.name == 'timer_global_path_period':
                self.timer_global_path.cancel()
                self.timer_global_path = self.create_timer(param.value, self.timer_global_path_callback)
                self.get_logger().info(f"Global path timer period set to {param.value} seconds")
            elif param.name == 'timer_mission_period':
                self.timer_mission.cancel()
                self.timer_mission = self.create_timer(param.value, self.timer_mission_callback)
                self.get_logger().info(f"Mission timer period set to {param.value} seconds")
            elif param.name == 'timer_monitoring_period':
                self.timer_monitoring.cancel()
                self.timer_monitoring = self.create_timer(param.value, self.timer_monitoring_pub_callback)
                self.get_logger().info(f"Monitoring timer period set to {param.value} seconds")
            elif param.name == 'return_hold_time':
                self.return_hold_time = float(param.value)
                self.get_logger().info(f"Return hold time set to {self.return_hold_time} seconds")

        result.successful = True
        return result
    
    def timer_start(self):
        self.get_logger().info("DroneManager Timer Started")
        timer_period_ocm =          self.get_parameter("timer_ocm_period").get_parameter_value().double_value  
        timer_period_uwb =          self.get_parameter("timer_uwb_period").get_parameter_value().double_value  
        timer_period_global_path =  self.get_parameter("timer_global_path_period").get_parameter_value().double_value  
        timer_period_mission =      self.get_parameter("timer_mission_period").get_parameter_value().double_value  
        timer_period_monitoring =   self.get_parameter("timer_monitoring_period").get_parameter_value().double_value  
        self.timer_uwb =            self.create_timer(timer_period_uwb, self.timer_uwb_callback)
        self.timer_global_path =    self.create_timer(timer_period_global_path, self.timer_global_path_callback)
        self.timer_mission =        self.create_timer(timer_period_mission, self.timer_mission_callback)
        self.timer_ocm =            self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        self.timer_monitoring =     self.create_timer(timer_period_monitoring, self.timer_monitoring_pub_callback)

    def get_desired_formation(self, side_length):
        n = len(self.system_id_list)
        R = side_length / (2.0 * math.sin(math.pi / n))
        desired_formation = []
        for i in range(n):
            theta = 2.0 * math.pi * i / n
            x = R * math.cos(theta)
            y = R * math.sin(theta)
            z = -self.mission_zlevel
            desired_formation.append((x, y, z))
        msg = "Formation Desired Positions:\n" + "\n".join(f"  {pos}" for pos in desired_formation)
        self.get_logger().info(msg)
        return desired_formation
    
    def initiate_drone_manager(self):
        self.change_mode(Mode.QHAC)
        self.change_ocm_msg_position()
        self.agent_uwb_range_dic.clear()

    ## Timer callback ##
    def timer_ocm_callback(self):
        self.ocm_publisher.publish(self.ocm_msg)
    
    def timer_global_path_callback(self):
        if len(self.global_path) == 0 or not self.mode_handler.is_in_mode(Mode.SEARCH):
            return
        else:
            traj_setpoint_msg = TrajectorySetpointMsg()
            traj_setpoint_msg.position[0] = self.global_path[0][0]
            traj_setpoint_msg.position[1] = self.global_path[0][1]
            traj_setpoint_msg.position[2] = self.global_path[0][2]
            dx = self.global_path[0][0] - self.monitoring_msg.pos_x
            dy = self.global_path[0][1] - self.monitoring_msg.pos_y
            desired_yaw = math.atan2(dy, dx)
            traj_setpoint_msg.yaw = desired_yaw
            self.traj_setpoint_publisher.publish(traj_setpoint_msg)
            if self.remain_distance(current_pos = [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y],
                                    target_pos = [self.global_path[0][0], self.global_path[0][1]]
                                    ) <= self.global_path_threshold:    
                self.global_path.pop(0)
                self.get_logger().error("WayPoint Arrived")
    
    def timer_uwb_callback(self):
        uwb_pub_msg = Ranging()
        uwb_pub_msg.header.frame_id            = "map"
        now_timestamp = self.get_clock().now().to_msg().sec
        uwb_pub_msg.header.stamp.sec = self.gcs_timestamp.stamp.sec + ( now_timestamp - self.init_timestamp )
        uwb_pub_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        uwb_pub_msg.anchor_id                  = self.system_id
        if self.uwb_sub_msg.range != -1:
            distance = self.uwb_sub_msg.range / 1000.0
            height   = self.monitoring_msg.pos_z
            square_diff = max(distance**2 - height**2, 0)
            uwb_pub_msg.range              = int(math.sqrt(square_diff) * 1000)
        else:
            uwb_pub_msg.range                  = self.uwb_sub_msg.range
        uwb_pub_msg.seq                        = self.uwb_sub_msg.seq
        uwb_pub_msg.rss                        = self.uwb_sub_msg.rss
        uwb_pub_msg.error_estimation           = float(self.mode_handler.get_mode().value)
        uwb_pub_msg.anchor_pose.position.x     = self.monitoring_msg.pos_x
        uwb_pub_msg.anchor_pose.position.y     = self.monitoring_msg.pos_y
        uwb_pub_msg.anchor_pose.position.z     = self.monitoring_msg.pos_z
        uwb_pub_msg.anchor_pose.orientation.x  = self.monitoring_msg.ref_lat
        uwb_pub_msg.anchor_pose.orientation.y  = self.monitoring_msg.ref_lon
        uwb_pub_msg.anchor_pose.orientation.z  = self.monitoring_msg.ref_alt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        
        self.uwb_ranging_publisher.publish(uwb_pub_msg)
        self.agent_uwb_range_dic[f'{self.system_id}'] = uwb_pub_msg
    
    def timer_monitoring_pub_callback(self):
        self.monitoring_publisher.publish(self.monitoring_msg)

    #### Mission Progress ####
    def timer_mission_callback(self):
        if len(self.takeoff_offset_dic) != 4:
            self.calculate_takeoff_offset()
            return
        self.update_uwb_data_list()
        self.particle_step()
        if len(self.uwb_data_list) >= 3:
            self.share_target()
        self.formation_move_agent()
        if self.mode_handler.is_in_mode(Mode.COLLECTION):
            self.handle_COLLECTION()
        if self.mode_handler.is_in_mode(Mode.RETURN):
            self.handle_RETURN()
        if self.mode_handler.is_in_mode(Mode.COMPLETED):  
            self.handle_COMPLETED()        

    ## Sub callback ##
    def monitoring_callback(self, msg):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        self.monitoring_msg = msg
        
    def uwb_msg_callback(self, msg):
        self.uwb_sub_msg = msg
        
    def global_path_callback(self, msg):
        self.get_logger().error("handle_global_path subscription called")
        self.global_path = []
        for point in msg.waypoints:
            self.global_path.append([
                point.position[0],
                point.position[1],
                point.position[2],
            ])
            self.mission_zlevel = - point.position[2]
    
    def timestamp_callback(self, msg):
        self.get_logger().info("System Time Synchronize.")
        self.gcs_timestamp = msg

    def mode_change_callback(self, msg):
        self.get_logger().info(f"DroneManager {self.system_id} : Mode Change Request : {Mode(msg.data)}")
        self.change_mode(Mode(msg.data))

    ## Particle Filter ##
    def particle_step(self):        
        ## It should run only when the mode is SERACH or HAVE_TARGET ##
        if not (self.mode_handler.is_in_mode(Mode.SEARCH) or self.mode_handler.is_in_mode(Mode.HAVE_TARGET) or self.mode_handler.is_in_mode(Mode.CONVERGED)):
            return
        if len(self.uwb_data_list) <= 0:
            if not self.mode_handler.is_in_mode(Mode.SEARCH):
                self.change_mode(Mode.SEARCH)
            return
        
        sensor_positions = [row[0] for row in self.uwb_data_list]
        measurements = [row[1] for row in self.uwb_data_list]
        noise_stds = [row[2] for row in self.uwb_data_list]
        
        self.particle_filter.step(sensor_positions, measurements, noise_stds, [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y])
        self.particle = self.particle_filter.particles
        
        self.target = self.particle_filter.estimate()
        estimate = [self.target[0], self.target[1]]
        self.publish_target(estimate)
        if self.particle is not None:
            self.publish_particle_cloud(self.particle)
        if self.mode_handler.is_in_mode(Mode.SEARCH):
            self.change_mode(Mode.HAVE_TARGET)
            
    
    def share_target(self):
        ## It should run only when the mode is HAVE_TARGET ##
        if not self.mode_handler.is_in_mode(Mode.HAVE_TARGET) and not self.mode_handler.is_in_mode(Mode.CONVERGED):
            return
        targets = []
        for key, value in self.agent_target_dic.items():
            if value is not None:
                ref_llh = [self.monitoring_msg.ref_lat,
                   self.monitoring_msg.ref_lon,
                   self.monitoring_msg.ref_alt]
                target = LLH2NED([value.position[0], value.position[1], 0], ref_llh)
                targets.append(target)
                # self.get_logger().info(f"target:, {target}")
        if len(targets):
            self.particle_filter.inject_shared([[t[0], t[1]] for t in targets])
            for key, value in self.agent_target_dic.items():
                self.agent_target_dic[key] = None

    def update_uwb_data_list(self):
        self.uwb_data_list.clear()
        for key, value in self.agent_uwb_range_dic.items():
            if value.range / 1000 <= self.uwb_threshold and value.range != -1:
                self.uwb_data_list.append([
                    [value.anchor_pose.position.x + self.takeoff_offset_dic[key][0],
                     value.anchor_pose.position.y + self.takeoff_offset_dic[key][1]],
                     value.range / 1000,
                     value.range / 1000 * 0.03,
                ])
    
    ## Formation & Repulsion
    def formation_move_agent(self):
        ## It should run only when the mode is HAVE_TARGET ##
        if not self.mode_handler.is_in_mode(Mode.HAVE_TARGET) and not self.mode_handler.is_in_mode(Mode.CONVERGED):
            return
        agents_pos = [None]*len(self.system_id_list)
        for key, value in self.agent_uwb_range_dic.items():
            idx = int(key)-1 
            agents_pos[idx] = [
                value.anchor_pose.position.x + self.takeoff_offset_dic[key][0],
                value.anchor_pose.position.y + self.takeoff_offset_dic[key][1],
                value.anchor_pose.position.z
            ]
        grad_formation = self.f_formation.compute(agents_pos)[self.system_id-1]
        ## Check if formation is converged ##
        err1, err2, err3 = self.f_formation.get_error(agents_pos)
        # self.get_logger().info(f"DroneManager {self.system_id} : shape_err, pair_err, scale_err: {err1}, {err2}, {err3}")
        grad_repulsion = self.f_repulsion.compute(agents_pos)[self.system_id-1]
        grad_target = self.f_target.compute(
            current_pos=[
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
                ],
            target=[
                self.target[0],
                self.target[1],
                - self.mission_zlevel,
                ]
            )
        w_repulsion, w_target, w_formation = self.compute_weight()
        total_grad = (
            - w_formation * grad_formation
            + w_repulsion * grad_repulsion
            - w_target    * grad_target
        )
        grad_norm = np.linalg.norm(total_grad)
        # self.get_logger().info(f"DroneManager {self.system_id} : grad_norm : {grad_norm}")
        self.is_formation_converged(grad_norm)
        total_grad = np.nan_to_num(total_grad)
        result_direc = self.set_direction(total_grad)
        speed = min(1000, np.linalg.norm(total_grad))
        dir_safe = np.nan_to_num(result_direc, nan=0.0, posinf=0.0, neginf=0.0)
        speed_safe = 0.0 if not np.isfinite(speed) else speed 
        current_pos=np.array([
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
                ])
        dt = 0.04
        next_pos = current_pos + dir_safe * speed_safe * dt
        next_pos[2] = min(next_pos[2], -0.1)
        setpoint = TrajectorySetpointMsg()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        setpoint.position = [float(next_pos[0]), float(next_pos[1]), float(next_pos[2])]
        setpoint.yaw = self.desired_yaw
        direction = TrajectorySetpointMsg()
        direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        self.total_gradient_publisher.publish(direction)
        self.traj_setpoint_publisher.publish(setpoint)

    def is_formation_converged(self, grad_norm):
        if grad_norm < self.tol:
            self.change_mode(Mode.CONVERGED)
        if all(v.error_estimation == Mode.CONVERGED.value for v in self.agent_uwb_range_dic.values()):
            # self.get_logger().info(f"DroneManager {self.system_id} : Formation Converged")
            if self.system_id == self.system_id_list[0]:
                dx = self.target[0] - self.monitoring_msg.pos_x
                dy = self.target[1] - self.monitoring_msg.pos_y
                desired_yaw = math.atan2(dy, dx)
                self.desired_yaw = float(desired_yaw)
                self.change_mode(Mode.RETURN, delay_seconds=self.return_hold_time)
            else:
                self.change_mode(Mode.RETURN, delay_seconds=self.return_hold_time)
            return
        
    def compute_weight(self):
        if self.have_target:
            current_pos = np.array([self.monitoring_msg.pos_x,
                       self.monitoring_msg.pos_y])
            target_pos = np.array([self.target[0],
                          self.target[1]])
            dist = np.linalg.norm(current_pos - target_pos)
            if dist > self.target_bound:
                return self.weight_table[1]
            else:
                return self.weight_table[2]
        else:
            return self.weight_table[0]

    def set_direction(self, vec):
        norm = np.linalg.norm(vec)
        if norm > 1e-8:
            return vec / norm

    ## Mode Handlers ##
    def handle_COMPLETED(self):
        if self.remain_distance(current_pos = [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y, self.monitoring_msg.pos_z],
                                target_pos = [0.0, 0.0, 0.0]
                                ) <= 0.5:
            self.get_logger().info(f"[Drone {self.system_id}] Completed Mission")
            self.mode_handler.change_mode(Mode.DONE)
        cmd = VehicleCommandMsg()
        cmd.target_system = self.system_id
        cmd.command = 21  # MAV_CMD_NAV_LAND
        cmd.from_external = True
        self.vehicle_command_publisher.publish(cmd)

    def handle_RETURN(self):
        agents_pos = [None] * len(self.system_id_list)
        for key, value in self.agent_uwb_range_dic.items():
            idx = int(key) - 1
            agents_pos[idx] = [
                value.anchor_pose.position.x + self.takeoff_offset_dic[key][0],
                value.anchor_pose.position.y + self.takeoff_offset_dic[key][1],
                value.anchor_pose.position.z
            ]

        # Compute repulsion and target gradients for this drone
        grad_repulsion = self.f_repulsion.compute(agents_pos)[self.system_id - 1]
        grad_target = self.f_target.compute(
            current_pos=[
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
            ],
            target=[
                0.0,
                0.0,
                -self.mission_zlevel,
            ]
        )

        # Compute weights for repulsion and target
        w_repulsion, w_target = self.compute_weight()[:2]

        # Combine gradients into a total gradient for velocity command
        total_grad = (
            + w_repulsion * grad_repulsion
            - w_target    * grad_target
        )
        total_grad = np.nan_to_num(total_grad)

        # Normalize direction and compute speed
        result_direc = self.set_direction(total_grad)
        speed = min(1000, np.linalg.norm(total_grad))
        dir_safe = np.nan_to_num(result_direc, nan=0.0, posinf=0.0, neginf=0.0)
        speed_safe = 0.0 if not np.isfinite(speed) else speed

        # Compute next position based on current position, direction, and speed
        current_pos = np.array([
            self.monitoring_msg.pos_x,
            self.monitoring_msg.pos_y,
            self.monitoring_msg.pos_z,
        ])
        dt = 0.04  # Time step
        next_pos = current_pos + dir_safe * speed_safe * dt
        next_pos[2] = min(next_pos[2], -0.1)

        # Publish trajectory setpoint
        setpoint = TrajectorySetpointMsg()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        setpoint.position = [
            float(next_pos[0]), float(next_pos[1]), float(next_pos[2])
        ]
        direction = TrajectorySetpointMsg()
        direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        self.total_gradient_publisher.publish(direction)
        self.traj_setpoint_publisher.publish(setpoint)

        # If within 0.5m of home (0,0), switch to COMPLETED
        if self.remain_distance(
            current_pos=[self.monitoring_msg.pos_x, self.monitoring_msg.pos_y],
            target_pos=[0.0, 0.0]
        ) <= 0.5:
            self.get_logger().info(f"[Drone {self.system_id}] Return Completed")
            self.change_mode(Mode.COMPLETED)
            self.get_logger().info(f"[Drone {self.system_id}] Landing command sent.")

    def handle_COLLECTION(self):
        if self.handle_flag:
            return
        agents_pos = [None]*len(self.system_id_list)
        for key, value in self.agent_uwb_range_dic.items():
            idx = int(key)-1 
            agents_pos[idx] = [
                value.anchor_pose.position.x + self.takeoff_offset_dic[key][0],
                value.anchor_pose.position.y + self.takeoff_offset_dic[key][1],
                value.anchor_pose.position.z
            ]
        grad_repulsion = self.f_repulsion.compute(agents_pos)[self.system_id-1]
        collection_target=[]
        if self.collection_step == 0:
            collection_target = [self.target[0], self.target[1], - (self.mission_zlevel + 4.0)]
        elif self.collection_step == 1:
            collection_target = [self.target[0], self.target[1], - (0.2)]
        elif self.collection_step == 2:
            collection_target = [self.target[0], self.target[1], - (self.mission_zlevel + 4.0)]
        grad_target = self.f_target.compute(
            current_pos=[
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
                ],
            target = collection_target
            )
        w_repulsion, w_target = self.compute_weight()[:2]
        total_grad = (
            + w_repulsion * grad_repulsion
            - w_target    * grad_target * 5
        )
        total_grad = np.nan_to_num(total_grad)
        result_direc = self.set_direction(total_grad)
        speed = min(1000, np.linalg.norm(total_grad))
        dir_safe = np.nan_to_num(result_direc, nan=0.0, posinf=0.0, neginf=0.0)
        speed_safe = 0.0 if not np.isfinite(speed) else speed 
        current_pos=np.array([
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
                ])
        dt = 0.04
        next_pos = current_pos + dir_safe * speed_safe * dt
        next_pos[2] = min(next_pos[2], -0.1)
        setpoint = TrajectorySetpointMsg()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        setpoint.position = [float(next_pos[0]), float(next_pos[1]), float(next_pos[2])]
        direction = TrajectorySetpointMsg()
        direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        self.total_gradient_publisher.publish(direction)
        self.traj_setpoint_publisher.publish(setpoint)
        
        remain_distance = np.linalg.norm(
            np.array([self.monitoring_msg.pos_x, self.monitoring_msg.pos_y, self.monitoring_msg.pos_z]) -
            np.array([collection_target[0], collection_target[1], collection_target[2]])
        )
        if remain_distance <= 0.2:
            if self.collection_step == 0:
                self.collection_step = 1
                self.get_logger().info(f"DroneManager {self.system_id} : Collection Step 1")
            elif self.collection_step == 1:
                self.collection_step = 2
                self.get_logger().info(f"DroneManager {self.system_id} : Collection Step 2")
            elif self.collection_step == 2:
                self.collection_step = 0
                self.get_logger().info(f"DroneManager {self.system_id} : Collection Completed")
                self.handle_flag = True
                self.change_mode(Mode.RETURN, delay_seconds=5.0)

    ## Make and return callback
    def make_uwb_range_callback(self, sys_id):
        self.get_logger().info(f"DroneManager {self.system_id} : Create Drone{sys_id} UWB Range Subscriber")
        def callback(msg):
            self.agent_uwb_range_dic[f'{sys_id}'] = msg
        return callback
    
    def make_target_callback(self, sys_id):
        self.get_logger().info(f"DroneManager {self.system_id} : Create Drone{sys_id} Target Subscriber")
        def callback(msg):
            self.agent_target_dic[f'{sys_id}'] = msg
        return callback
    
    ## Publisher ##
    def publish_target(self, target):
        target = TrajectorySetpointMsg()
        target_pos_ned = [self.target[0],
                          self.target[1],
                          0.1]
        ref_llh = [self.monitoring_msg.ref_lat,
                   self.monitoring_msg.ref_lon,
                   self.monitoring_msg.ref_alt]
        target_pos_llh = NED2LLH(NED=target_pos_ned, ref_LLH=ref_llh)
        target.position[0] = target_pos_llh[0]
        target.position[1] = target_pos_llh[1]
        self.target_publisher.publish(target)

    def publish_particle_cloud(self, particles: np.ndarray):
        header = std_msgs.Header()
        header.frame_id = 'map'
        header.stamp    = self.get_clock().now().to_msg()

        points = [(float(x), float(y), 0.0) for x,y in particles]
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.particle_publisher.publish(cloud_msg)

    ## OCM Msg ##
    def change_ocm_msg_position(self):
        self.ocm_msg.position = True
        self.ocm_msg.velocity = False
        self.ocm_msg.acceleration = False
        self.ocm_msg.attitude = False
        self.ocm_msg.body_rate = False
        self.ocm_msg.actuator = False
        self.get_logger().info(f"DroneManager {self.system_id} : OCM changed : Position")

    def change_ocm_msg_velocity(self):
        self.ocm_msg.position = False
        self.ocm_msg.velocity = True
        self.ocm_msg.acceleration = False
        self.ocm_msg.attitude = False
        self.ocm_msg.body_rate = False
        self.ocm_msg.actuator = False
        self.get_logger().info(f"DroneManager {self.system_id} : OCM changed : Velocity")

    ## Utility ##
    def calculate_takeoff_offset(self):
        self.takeoff_offset_dic.clear()
        ref_LLH = [self.monitoring_msg.ref_lat, self.monitoring_msg.ref_lon, self.monitoring_msg.ref_alt]
        for key, value in self.agent_uwb_range_dic.items():
            try:
                if value.anchor_pose.orientation.x == 0.0:
                    raise ValueError("Orientation.x is zero")
                LLH = [value.anchor_pose.orientation.x, value.anchor_pose.orientation.y, value.anchor_pose.orientation.z]
                if any(math.isnan(val) for val in LLH) or any(math.isnan(val) for val in ref_LLH):
                    raise ValueError("NaN in coordinates")
                NED = LLH2NED(LLH, ref_LLH)
                self.takeoff_offset_dic[f'{key}'] = NED
            except Exception as e:
                pass
                # self.get_logger().warn(f"Key {key} skipped: {e}")

    def remain_distance(self, current_pos, target_pos):
        return math.sqrt((current_pos[0] - target_pos[0])**2 + (current_pos[1] - target_pos[1])**2)
        
    def change_mode(self, mode, delay_seconds= None):
        if delay_seconds is None:
            result = self.mode_handler.change_mode(mode)
            if result == -1:
                pass
                self.get_logger().warn(f"DroneManager {self.system_id} : Mode Change Failed to {Mode(mode)}")
            else:
                
                self.get_logger().info(f"DroneManager {self.system_id} : Mode Changed to {Mode(mode)}")
        else:
            result = self.mode_handler.change_mode_delay(mode, delay_seconds)
            if result == -1:
                pass
                self.get_logger().warn(f"DroneManager {self.system_id} : Mode Change Delay Failed to {Mode(mode)}")
            else:
                
                self.get_logger().info(f"DroneManager {self.system_id} : Mode Change Delay Start... {Mode(mode)} | {delay_seconds} seconds")

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

    dronemanager = DroneManager()

    rclpy.spin(dronemanager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dronemanager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()