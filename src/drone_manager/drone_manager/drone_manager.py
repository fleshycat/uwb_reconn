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

from std_msgs.msg import Header

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
    DETECT = 2
    HAVE_TARGET = 3

class DroneManager(Node):
    def __init__(self):
        super().__init__("drone_manager")
        self.declare_parameter('system_id', 1)
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.get_logger().info(f"Configure DroneManager {self.system_id}")
        self.declare_parameter('system_id_list', [1,2,3,4])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        
        self.mode = Mode.QHAC
        self.topic_prefix_manager = f"drone{self.system_id}/manager/"  #"drone1/manager/"
        self.topic_prefix_fmu = f"drone{self.system_id}/fmu/"          #"drone1/fmu/"
        self.topic_prefix_uwb = f"drone{self.system_id}/uwb/ranging"
        
        self.monitoring_msg = Monitoring()
        self.uwb_sub_msg = RangingDiff()
        self.global_path = []
        self.global_path_threshold = 0.1
        self.takeoff_offset_dic = {}
        self.agent_uwb_range_dic = {f'{i}':Ranging() for i in self.system_id_list}
        
        ## Publisher ##
        self.ocm_publisher = self.create_publisher(OffboardControlMode, f'{self.topic_prefix_fmu}in/offboard_control_mode', qos_profile_sensor_data)                    #"drone1/fmu/in/offboard_control_mode"
        self.uwb_ranging_publisher = self.create_publisher(Ranging, f'{self.topic_prefix_manager}out/ranging', qos_profile_sensor_data)
        self.traj_setpoint_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_fmu}in/trajectory_setpoint', qos_profile_sensor_data)
        self.target_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_manager}out/target', qos_profile_sensor_data)
        self.particle_publisher = self.create_publisher(PointCloud2, f'{self.topic_prefix_manager}out/particle_cloud', qos_profile_sensor_data)

        ## Subscriber ##
        self.uwb_subscriber = self.create_subscription(RangingDiff, self.topic_prefix_uwb, self.uwb_msg_callback, qos_profile_sensor_data)
        self.monitoring_subscriber = self.create_subscription(Monitoring, f'{self.topic_prefix_fmu}out/monitoring', self.monitoring_callback, qos_profile_sensor_data)  #"drone1/fmu/out/monitoring"
        self.timestamp_subscriber = self.create_subscription(Header, f'qhac/manager/in/timestamp',self.timestamp_callback, 10)
        self.global_path_subscriber = self.create_subscription(GlobalPathMsg, f'{self.topic_prefix_manager}in/global_path', self.global_path_callback, 10)
        self.agent_monitoring_subscribers = [
            self.create_subscription(Ranging, f'drone{i}/manager/out/ranging', self.make_monitoring_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list if i != self.system_id
        ]
        
        ## OCM Msg ##
        self.ocm_msg = OffboardControlMode()
        self.ocm_msg.position = True
        self.ocm_msg.velocity = False
        self.ocm_msg.acceleration = False
        self.ocm_msg.attitude = False
        self.ocm_msg.body_rate = False
        self.ocm_msg.actuator = False
        
        ## Potential Field ##
        desired_formation = [(0,0),(5,0),(5,5),(0,5),]
        self.f_formation = FormationForce(desired_positions = desired_formation,
                                        k_scale=1.0,
                                        k_pair=1.0,
                                        k_shape=10.0)
        self.f_repulsion = RepulsionForce(n_agents=len(self.system_id_list),
                                        c_rep=0.05,
                                        cutoff=5,
                                        p=2)
        self.f_target = TargetForce([0,0], k_mission=0.5)
        length = np.linalg.norm(np.array(desired_formation[0]) - np.array(desired_formation[1]))
        self.target_bound = np.sqrt(2) * length
        self.weight_table = [(4,2,1),
                             (4,1,2),
                             (4,1,1),]
        
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
        timer_period_mission = 0. # 25hz
        self.timer_mission = self.create_timer(timer_period_mission, self.timer_mission_callback)
        
        self.gcs_timestamp = Header()
        self.init_timestamp = self.get_clock().now().to_msg().sec
        
        self.initiate_drone_manager()
    
    def initiate_drone_manager(self):
        pass
        
    ## Timer callback ##
    def timer_ocm_callback(self):
        self.ocm_publisher.publish(self.ocm_msg)
    
    def timer_global_path_callback(self):
        # if self.mode != Mode.EGO:
        #     return
        if len(self.global_path) == 0:
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
    
    def timer_mission_callback(self):
        if len(self.takeoff_offset_dic) != 4:
            self.calculate_takeoff_offset()
            return
        self.update_uwb_data_list()
        self.particle_step()
        
    
    def timer_uwb_callback(self):
        uwb_pub_msg = Ranging()
        uwb_pub_msg.header.frame_id            = "map"
        now_timestamp = self.get_clock().now().to_msg().sec
        uwb_pub_msg.header.stamp.sec = self.gcs_timestamp.stamp.sec + ( now_timestamp - self.init_timestamp )
        uwb_pub_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        uwb_pub_msg.anchor_id                  = self.system_id
        uwb_pub_msg.range                      = self.uwb_sub_msg.range
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
                point.jerk[0]
            ])
    
    def particle_step(self):        
        if len(self.uwb_data_list) <= 0:
            self.get_logger().info(f"DroneManager{self.system_id}: No uwb data available")
            self.have_target = False
            return
        
        sensor_positions = [row[0] for row in self.uwb_data_list]
        measurements = [row[1] for row in self.uwb_data_list]
        noise_stds = [row[2] for row in self.uwb_data_list]
        
        self.particle_filter.step(sensor_positions, measurements, noise_stds, [self.monitoring_msg.pos_x,self.monitoring_msg.pos_y])
        self.particle = self.particle_filter.particles
        
        self.target = self.particle_filter.estimate()
        self.estimate = [self.target[0], self.target[1]]
        self.publish_target(self.estimate)
        if self.particle is not None:
            self.publish_particle_cloud(self.particle)
        self.have_target = True
    
    def update_uwb_data_list(self):
        self.uwb_data_list.clear()
        for i in self.system_id_list:
            if self.agent_uwb_range_dic[f'{i}'].range / 1000 <= self.uwb_threshold:
                self.uwb_data_list.append([
                    [self.agent_uwb_range_dic[f'{i}'].anchor_pose.position.x + self.takeoff_offset_dic[f'{i}'][0], 
                     self.agent_uwb_range_dic[f'{i}'].anchor_pose.position.y + self.takeoff_offset_dic[f'{i}'][1]],
                    self.agent_uwb_range_dic[f'{i}'].range / 1000,
                    self.agent_uwb_range_dic[f'{i}'].range / 1000 * 0.03,
                    ])
                
    def make_monitoring_callback(self, sys_id):
        self.get_logger().info(f"DroneManager {self.system_id} : Create Drone{sys_id} Monitoring Subscriber")
        def callback(msg):
            self.agent_uwb_range_dic[f'{sys_id}'] = msg
        return callback
    
    def timestamp_callback(self, msg):
        self.get_logger().info("System Time Synchronize.")
        self.gcs_timestamp = msg
        
    def remain_distance(self, current_pos, target_pos):
        return math.sqrt((current_pos[0] - target_pos[0])**2 + (current_pos[1] - target_pos[1])**2)

    def calculate_takeoff_offset(self):
        self.takeoff_offset_dic.clear()
        ref_LLH = [self.monitoring_msg.ref_lat, self.monitoring_msg.ref_lon, self.monitoring_msg.ref_alt]
        # self.get_logger().info(f"DroneManager {self.system_id} : ref_LLH : {ref_LLH}")
        for key, value in self.agent_uwb_range_dic.items():
            if value.anchor_pose.orientation.x == 0.0:
                return
            LLH = [value.anchor_pose.orientation.x, value.anchor_pose.orientation.y, value.anchor_pose.orientation.z]
            # self.get_logger().info(f"Key : {key}, LLH : {LLH}")
            if any(math.isnan(val) for val in LLH) or any(math.isnan(val) for val in ref_LLH):
                # self.get_logger().warn("LLH ref_LLH NaN.")
                return
            NED = LLH2NED(LLH, ref_LLH)
            self.takeoff_offset_dic[f'{key}'] = NED
        # print("self.takeoff_offset:", self.takeoff_offset_dic, sep="\n")
        return
    
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
        target.velocity[0] = target_pos_ned[0]
        target.velocity[1] = target_pos_ned[1]
        self.target_publisher.publish(target)

    def publish_particle_cloud(self, particles: np.ndarray):
        header = std_msgs.Header()
        header.frame_id = 'map'
        header.stamp    = self.get_clock().now().to_msg()

        points = [(float(x), float(y), 0.0) for x,y in particles]
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.particle_publisher.publish(cloud_msg)


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

    dronemanager = DroneManager()

    rclpy.spin(dronemanager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dronemanager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()