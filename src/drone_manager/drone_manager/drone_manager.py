import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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

from enum import Enum
import math
import numpy as np

class Mode(Enum):
    QHAC = 0
    SEARCH = 1
    HAVE_TARGET = 2
    COLLECTION = 3
    RETURN = 4
    COMPLETED = 5
class DroneManager(Node):
    def __init__(self):
        super().__init__("drone_manager")
        self.declare_parameter('system_id', 1)
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.get_logger().info(f"Configure DroneManager {self.system_id}")
        self.declare_parameter('system_id_list', [1,2,3,4])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        
        self.topic_prefix_manager = f"drone{self.system_id}/manager/"  #"drone1/manager/"
        self.topic_prefix_fmu = f"drone{self.system_id}/fmu/"          #"drone1/fmu/"
        self.topic_prefix_uwb = f"drone{self.system_id}/uwb/ranging"
        
        self.monitoring_msg = Monitoring()
        self.uwb_sub_msg = RangingDiff()
        self.mode = Mode.QHAC
        self.global_path = []
        self.global_path_threshold = 0.1
        self.takeoff_offset_dic = {}
        self.agent_uwb_range_dic = {f'{i}':Ranging() for i in self.system_id_list}
        self.agent_target_dic = {}
        self.mission_zlevel = 3.0
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
        self.mode_change_subscriber = self.create_subscription(UInt8, f'{self.topic_prefix_manager}in/mode_change', self.mode_change_callback, qos_profile_sensor_data)
        self.agent_uwb_range_subscribers = [
            self.create_subscription(Ranging, f'drone{i}/manager/out/ranging', self.make_uwb_range_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list if i != self.system_id
        ]
        self.agent_target_subscribers = [
            self.create_subscription(TrajectorySetpointMsg, f'drone{i}/manager/out/target', self.make_target_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list if i != self.system_id
        ]
        
        ## OCM Msg ##
        self.ocm_msg = OffboardControlMode()
        
        ## Potential Field ##
        desired_formation = [(0,0,-self.mission_zlevel), 
                             (6,0,-self.mission_zlevel), 
                             (6,6,-self.mission_zlevel), 
                             (0,6,-self.mission_zlevel)]
        self.f_formation = FormationForce(desired_positions = desired_formation,
                                        k_scale=1.0,
                                        k_pair=1.0,
                                        k_shape=2.0,
                                        k_z=2.0)
        self.f_repulsion = RepulsionForce(n_agents=len(self.system_id_list),
                                        c_rep=3.0,
                                        cutoff=2.0,
                                        sigma=1.0)
        self.f_target = TargetForce([0,0], k_target=1.0)
        length = np.linalg.norm(np.array(desired_formation[0]) - np.array(desired_formation[1]))
        self.target_bound = np.sqrt(self.mission_zlevel**2 + length**2 / 2.0)
        self.weight_table = [(0,1,1),               ## w_repulsion, w_target, w_formation
                             (4,1,0),               ## not in target bound
                             (4,1,2),]              ## in target bound
        
        ## Particle Filter ##
        self.num_particles = 1000
        self.particle_filter = ParticleFilter(num_particles=self.num_particles)
        self.target = []
        self.have_target = False
        self.uwb_data_list = []
        self.uwb_threshold = 10.0
        
        ## Timer ##
        timer_period_ocm = 0.1
        self.timer_ocm = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        timer_period_uwb = 0.04 # 25hz
        self.timer_uwb = self.create_timer(timer_period_uwb, self.timer_uwb_callback)
        timer_period_global_path = 0.1
        self.timer_global_path = self.create_timer(timer_period_global_path, self.timer_global_path_callback)
        timer_period_mission = 0.04 # 25hz
        self.timer_mission = self.create_timer(timer_period_mission, self.timer_mission_callback)
        timer_period_monitoring = 0.02 # 50hz
        self.timer_monitoring = self.create_timer(timer_period_monitoring, self.timer_monitoring_pub_callback)
        
        self.gcs_timestamp = Header()
        self.init_timestamp = self.get_clock().now().to_msg().sec
        
        self.initiate_drone_manager()
    
    def initiate_drone_manager(self):
        self.change_mode(Mode.QHAC)
        self.change_ocm_msg_position()
        self.agent_uwb_range_dic.clear()

    ## Timer callback ##
    def timer_ocm_callback(self):
        self.ocm_publisher.publish(self.ocm_msg)
    
    def timer_global_path_callback(self):
        if len(self.global_path) == 0 or self.mode != Mode.SEARCH:
            # self.get_logger().warn("Attempting to send a setpoint when the global path is empty ")
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
        uwb_pub_msg.error_estimation           = self.uwb_sub_msg.error_estimation
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
        if self.mode == Mode.COLLECTION:
            self.handle_COLLECTION()
        elif self.mode == Mode.RETURN:
            self.handle_RETURN()
        elif self.mode == Mode.COMPLETED:
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
        self.get_logger().info(f"DroneManager {self.system_id} : Mode Change Request : {msg.data}")
        if msg.data == Mode.SEARCH.value:
            self.change_mode(Mode.SEARCH)
        elif msg.data == Mode.HAVE_TARGET.value:
            self.change_mode(Mode.HAVE_TARGET)
        elif msg.data == Mode.COLLECTION.value:
            self.change_mode(Mode.COLLECTION)
        elif msg.data == Mode.RETURN.value:
            self.change_mode(Mode.RETURN)
        elif msg.data == Mode.COMPLETED.value:
            self.change_mode(Mode.COMPLETED)

    ## Particle Filter ##
    def particle_step(self):        
        ## It should run only when the mode is SERACH or HAVE_TARGET ##
        if not (self.mode == Mode.SEARCH or self.mode == Mode.HAVE_TARGET):
            return
        if len(self.uwb_data_list) <= 0:
            if self.mode != Mode.SEARCH:
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
        if self.mode == Mode.SEARCH:
            self.get_logger().info(f"DroneManager {self.system_id} : Particle Filter Estimate : {estimate}")
            self.change_mode(Mode.HAVE_TARGET)
    
    def share_target(self):
        ## It should run only when the mode is HAVE_TARGET ##
        if self.mode != Mode.HAVE_TARGET:
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
        if self.mode != Mode.HAVE_TARGET:
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
        err1, err2, err3, err4 = self.f_formation.get_error(agents_pos)
        self.get_logger().info(f"DroneManager {self.system_id} : shape_err, pair_err, scale_err, z_error : {err1}, {err2}, {err3}, {err4}")
        self.get_logger().info(f"DroneManager {self.system_id} : is_converged : {self.f_formation.is_converged(agents_pos, tol=0.1)}")
        if self.f_formation.is_converged(agents_pos, tol=0.001):
            self.get_logger().info(f"DroneManager {self.system_id} : Formation Converged")
            if self.system_id == self.system_id_list[0]:
                self.change_mode(Mode.COLLECTION)
            else:
                self.change_mode(Mode.RETURN)
            return
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

    def handle_COLLECTION(self):
        agents_pos = [None]*len(self.system_id_list)
        for key, value in self.agent_uwb_range_dic.items():
            idx = int(key)-1 
            agents_pos[idx] = [
                value.anchor_pose.position.x + self.takeoff_offset_dic[key][0],
                value.anchor_pose.position.y + self.takeoff_offset_dic[key][1],
                value.anchor_pose.position.z
            ]
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

        if self.remain_distance(current_pos = [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y],
                                    target_pos = [self.target[0], self.target[1]]
                                    ) <= 0.5:
            self.get_logger().info(f"DroneManager {self.system_id} : Collection Completed")
            self.change_mode(Mode.RETURN)
    
    def handle_RETURN(self):
        agents_pos = [None]*len(self.system_id_list)
        for key, value in self.agent_uwb_range_dic.items():
            idx = int(key)-1 
            agents_pos[idx] = [
                value.anchor_pose.position.x + self.takeoff_offset_dic[key][0],
                value.anchor_pose.position.y + self.takeoff_offset_dic[key][1],
                value.anchor_pose.position.z
            ]
        grad_repulsion = self.f_repulsion.compute(agents_pos)[self.system_id-1]
        grad_target = self.f_target.compute(
            current_pos=[
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
                ],
            target=[
                0.0,
                0.0,
                - self.mission_zlevel,
                ]
            )
        w_repulsion, w_target = self.compute_weight()[:2]
        total_grad = (
            + w_repulsion * grad_repulsion
            - w_target    * grad_target
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

        if self.remain_distance(current_pos = [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y],
                                    target_pos = [0.0, 0.0]
                                    ) <= 0.5:    
            self.get_logger().info(f"DroneManager {self.system_id} : Return Completed")
            self.change_mode(Mode.COMPLETED)
            
    def handle_COMPLETED(self):
        cmd  = VehicleCommandMsg()
        cmd.target_system = self.system_id
        cmd.command = 21  # MAV_CMD_NAV_LAND
        cmd.from_external = True
        self.vehicle_command_publisher.publish(cmd)

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

    def change_mode(self, mode):
        if mode == self.mode:
            return
        if mode == Mode.QHAC:
            self.change_ocm_msg_position()
        elif mode == Mode.SEARCH:                       ## Search Mode is agents searching for target
            self.have_target = False
        elif mode == Mode.HAVE_TARGET:                  ## Have Target Mode is agents have target and adjust formation
            self.have_target = True
            # self.change_ocm_msg_velocity()
        elif mode == Mode.COLLECTION:                   ## Collection Mode is agents collecting target
            pass
        elif mode == Mode.RETURN:                       ## Return Mode is agents returning to home position
            pass
        elif mode == Mode.COMPLETED:                      ## Landing Mode is agents landing
            pass
        self.mode = mode
        self.get_logger().info(f'Mode Changed : {mode}')
        

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