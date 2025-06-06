import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.srv import ModeChange, TrajectorySetpoint as TrajectorySetpointSrv, \
    VehicleCommand as VehicleCommandSrv, GlobalPath as GlobalPathSrv
from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, \
    OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, \
    VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor, GlobalPath as GlobalPathMsg

from uwb_msgs.msg import Ranging
from nlink_parser_ros2_interfaces.msg import LinktrackNodeframe2

## only for simulation ##
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from std_msgs.msg import Header, UInt8

from drone_manager.class_particle import ParticleFilter
from drone_manager.formation import FormationForce
from drone_manager.repulsion import RepulsionForce
from drone_manager.target import TargetForce
from drone_manager.jfi import JFiInterface
from drone_manager.mode_handler import ModeHandler, Mode

import math
import numpy as np
import struct

class DroneManager(Node):
    def __init__(self):
        super().__init__("drone_manager")

        # --- ROS2 parameter ---
        self.declare_parameter('system_id', 1)
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.get_logger().info(f"Configure DroneManager {self.system_id}")

        self.declare_parameter('system_id_list', [1, 2, 3, 4])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value

        # --- J-Fi Configuration ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.declare_parameter('baud_rate', 115200)
        baudrate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Create J-Fi Interface (receive callback: self._on_jfi_payload)
        try:
            self.jfi = JFiInterface(
                port_name=serial_port_name,
                baudrate=baudrate,
                receive_callback=self._on_jfi_payload
            )
            self.get_logger().info(f"Opened J-Fi serial on {serial_port_name}@{baudrate}")
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return

        # J-Fi seq (0~255)
        self.jfi_seq = 0

        # --- Topic prefix setting ---
        self.topic_prefix_manager = f"drone{self.system_id}/manager/"
        self.topic_prefix_fmu     = f"drone{self.system_id}/fmu/"
        self.topic_prefix_uwb     = f"drone{self.system_id}/uwb/ranging"

        # --- Internal Status Variables ---
        self.monitoring_msg      = Monitoring()
        self.uwb_sub_msg         = LinktrackNodeframe2()
        self.global_path         = []
        self.global_path_threshold = 0.1
        self.takeoff_offset_dic  = {}
        self.agent_uwb_range_dic = {f"{i}": Ranging() for i in self.system_id_list}
        self.agent_target_dic    = {}
        self.mission_zlevel      = 3.0
        self.direction           = TrajectorySetpointMsg()

        # --- Publisher ---
        self.ocm_publisher = self.create_publisher(
            OffboardControlMode,
            f"{self.topic_prefix_fmu}in/offboard_control_mode",
            qos_profile_sensor_data
        )
        self.traj_setpoint_publisher = self.create_publisher(
            TrajectorySetpointMsg,
            f"{self.topic_prefix_fmu}in/trajectory_setpoint",
            qos_profile_sensor_data
        )
        self.monitoring_publisher = self.create_publisher(
            Monitoring,
            f"{self.topic_prefix_manager}out/monitoring",
            qos_profile_sensor_data
        )
        # self.particle_publisher = self.create_publisher(
        #     PointCloud2,
        #     f'{self.topic_prefix_manager}out/particle_cloud',
        #     qos_profile_sensor_data
        # )
        # self.total_gradient_publisher = self.create_publisher(
        #     TrajectorySetpointMsg,
        #     f'{self.topic_prefix_manager}out/gradient',
        #     qos_profile_sensor_data
        # )
        # Optional
        self.uwb_ranging_publisher = self.create_publisher(
            Ranging,
            f"{self.topic_prefix_manager}out/ranging",
            qos_profile_sensor_data
        )
        # self.target_publisher = self.create_publisher(
        #     TrajectorySetpointMsg,
        #     f"{self.topic_prefix_manager}out/target",
        #     qos_profile_sensor_data
        # )
        self.vehicle_command_publisher = self.create_publisher(VehicleCommandMsg, f'{self.topic_prefix_fmu}in/vehicle_command', qos_profile_sensor_data)

        # --- Subscriber ---
        self.uwb_subscriber = self.create_subscription(
            LinktrackNodeframe2,
            f"drone{self.system_id}/nlink_linktrack_nodeframe2",
            self.uwb_msg_callback,
            qos_profile_sensor_data
        )
        self.monitoring_subscriber = self.create_subscription(
            Monitoring,
            f"{self.topic_prefix_fmu}out/monitoring",
            self.monitoring_callback,
            qos_profile_sensor_data
        )
        self.timestamp_subscriber = self.create_subscription(
            Header,
            "qhac/manager/in/timestamp",
            self.timestamp_callback,
            10
        )
        self.global_path_subscriber = self.create_subscription(
            GlobalPathMsg,
            f"{self.topic_prefix_manager}in/global_path",
            self.global_path_callback,
            10
        )
        self.mode_change_subscriber = self.create_subscription(UInt8, f'{self.topic_prefix_manager}in/mode_change', self.mode_change_callback, qos_profile_sensor_data)

        # --- OCM Msg ---
        self.ocm_msg = OffboardControlMode()

        # --- Potential Field ---
        desired_formation = [
            (0, 0, -self.mission_zlevel),
            (6, 0, -self.mission_zlevel),
            (6, 6, -self.mission_zlevel),
            (0, 6, -self.mission_zlevel),
        ]
        self.f_formation = FormationForce(
            desired_positions=desired_formation,
            k_scale=1.0, k_pair=1.0, k_shape=2.0, k_z=2.0
        )
        self.tol = 1.5
        self.f_repulsion = RepulsionForce(
            n_agents=len(self.system_id_list),
            c_rep=3.0, cutoff=2.0, sigma=1.0
        )
        self.f_target = TargetForce([0, 0], k_target=1.0)
        length = np.linalg.norm(np.array(desired_formation[0]) - np.array(desired_formation[1]))
        self.target_bound = np.sqrt(self.mission_zlevel**2 + length**2 / 2.0)
        self.weight_table = [
            (0, 1, 1),   # w_repulsion, w_target, w_formation
            (4, 1, 0),   # not in target bound
            (4, 1, 2),   # in target bound
        ]

        # --- Particle Filter ---
        self.num_particles = 1000
        self.particle_filter = ParticleFilter(num_particles=self.num_particles)
        self.target         = []
        self.have_target    = False
        self.uwb_data_list  = []
        self.uwb_threshold  = 10.0

        # --- ModeHandler --- 
        self.mode_handler = ModeHandler()
        self.handle_flag = False

        # --- Timer setting ---
        self.timer_ocm = self.create_timer(0.1, self.timer_ocm_callback)           # 10 Hz
        self.timer_uwb = self.create_timer(0.02, self.timer_uwb_callback)          # 50 Hz
        self.timer_global_path = self.create_timer(0.1, self.timer_global_path_callback)  # 10 Hz
        self.timer_mission = self.create_timer(0.04, self.timer_mission_callback)  # 25 Hz
        self.timer_monitoring = self.create_timer(0.02, self.timer_monitoring_pub_callback)  # 50 Hz

        self.gcs_timestamp = Header()
        self.init_timestamp = self.get_clock().now().to_msg().sec

        self.initiate_drone_manager()

    # --------------------------
    #  J-Fi Methods
    # --------------------------
    def send_uwb_data(self, uwb_pub_msg: Ranging):
        """
            Payload format for UWB ranging message:
          - B: system_id (uint8)
          - i: range (int32)
          - B: MODE  (uint8)
          - d*3: pos_x, pos_y, pos_z (double*3)
          - d*3: ori_x, ori_y, ori_z (double*3)
          - f: rss (float32)
          - f: error_estimation (float32)
        """
        fmt = '<B i B d d d d d d f f'
        payload = struct.pack(
            fmt,
            uwb_pub_msg.anchor_id,                  # uint8
            uwb_pub_msg.range,                      # int32
            uwb_pub_msg.seq,                        # uint8
            uwb_pub_msg.anchor_pose.position.x,     # double
            uwb_pub_msg.anchor_pose.position.y,     # double
            uwb_pub_msg.anchor_pose.position.z,     # double
            uwb_pub_msg.anchor_pose.orientation.x,  # double
            uwb_pub_msg.anchor_pose.orientation.y,  # double
            uwb_pub_msg.anchor_pose.orientation.z,  # double
            uwb_pub_msg.rss,                        # float32
            uwb_pub_msg.error_estimation            # float32
        )
        # Create a JFiProtocol packet
        self.jfi_seq = (self.jfi_seq + 1) & 0xFF
        self.jfi.send_packet(payload, seq=self.jfi_seq, sid=self.system_id)

    def send_target_data(self, target_msg: TrajectorySetpointMsg):
        """
            Payload format for J-Fi target message:
          - d*3: position_x, position_y, position_z (double*3)
        """
        fmt = '<d d d'
        payload = struct.pack(
            fmt,
            target_msg.position[0],  # double
            target_msg.position[1],  # double
            target_msg.position[2]   # double
        )

        # Create a JFiProtocol packet
        self.jfi_seq = (self.jfi_seq + 1) & 0xFF
        self.jfi.send_packet(payload, seq=self.jfi_seq, sid=self.system_id)

    def _on_jfi_payload(self, payload: bytes, seq_jfi: int, sid: int):
        if len(payload) == 62:
            # UWB payload: '<B i B d d d d d d f f'
            anchor_id, range, mode, \
            posx, posy, posz, \
            orix, oriy, oriz, \
            rss, err = struct.unpack('<B i B d d d d d d f f', payload)

            uwb_msg = Ranging()
            uwb_msg.header.stamp = self.get_clock().now().to_msg()
            uwb_msg.header.frame_id = "map"
            uwb_msg.anchor_id = anchor_id
            uwb_msg.range = range
            uwb_msg.seq = mode
            uwb_msg.anchor_pose.position.x = posx
            uwb_msg.anchor_pose.position.y = posy
            uwb_msg.anchor_pose.position.z = posz
            uwb_msg.anchor_pose.orientation.x = orix
            uwb_msg.anchor_pose.orientation.y = oriy
            uwb_msg.anchor_pose.orientation.z = oriz
            uwb_msg.rss = rss
            uwb_msg.error_estimation = err

            # Update the agent UWB range dictionary
            self.agent_uwb_range_dic[f"{sid}"] = uwb_msg

        elif len(payload) == 24:
            # Target payload: '<d d d'
            lat, lon, alt = struct.unpack('<d d d', payload)
            target_msg = TrajectorySetpointMsg()
            target_msg.position[0] = lat
            target_msg.position[1] = lon
            target_msg.position[2] = alt

            # Update the agent target dictionary
            self.agent_target_dic[f"{sid}"] = target_msg

        else:
            self.get_logger().warn(f"Undefined J-Fi payload length={len(payload)} (SID={sid})")
    
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

    # def timer_gradient_callback(self):
    #     if self.mode != Mode.HAVE_TARGET:
    #         return
    #     else:
    #         self.traj_setpoint_publisher.publish(self.direction)

    def timer_uwb_callback(self):
        # Create Ranging message
        uwb_pub_msg = Ranging()
        uwb_pub_msg.header.frame_id = "map"

        # Set the timestamp for the message
        now_timestamp = self.get_clock().now().to_msg().sec
        uwb_pub_msg.header.stamp.sec = self.gcs_timestamp.stamp.sec + (now_timestamp - self.init_timestamp )
        uwb_pub_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        
        # Set anchor ID
        uwb_pub_msg.anchor_id = self.system_id
        
        # Default values
        uwb_pub_msg.range = -1
        uwb_pub_msg.rss = 0.0
        uwb_pub_msg.error_estimation = 0.0

        # Find the node with ID 0 (Tag)
        for node in self.uwb_sub_msg.nodes:
            if node.id == 0:
                uwb_pub_msg.range = int(node.dis * 1000)    # m → mm
                uwb_pub_msg.rss = node.rx_rssi              # Received Signal Strength Indicator
                uwb_pub_msg.error_estimation = node.fp_rssi # First Path Received Signal Strength Indicator
                break

        if uwb_pub_msg.range != -1:
            distance = uwb_pub_msg.range / 1000.0                       # mm → m
            height = self.monitoring_msg.pos_z
            square_diff = max(distance**2 - height**2, 0)
            uwb_pub_msg.range   = int(math.sqrt(square_diff) * 1000)    # m → mm
        
        # Mode Value
        uwb_pub_msg.seq = self.mode_handler.get_mode().value

        # Drone NED position
        uwb_pub_msg.anchor_pose.position.x     = self.monitoring_msg.pos_x
        uwb_pub_msg.anchor_pose.position.y     = self.monitoring_msg.pos_y
        uwb_pub_msg.anchor_pose.position.z     = self.monitoring_msg.pos_z
        # Drone Ref LLH (RTK-GPS)
        uwb_pub_msg.anchor_pose.orientation.x  = self.monitoring_msg.ref_lat
        uwb_pub_msg.anchor_pose.orientation.y  = self.monitoring_msg.ref_lon
        uwb_pub_msg.anchor_pose.orientation.z  = self.monitoring_msg.ref_alt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        
        # Publish the UWB message
        self.send_uwb_data(uwb_pub_msg)
        self.uwb_ranging_publisher.publish(uwb_pub_msg)

        # Update the agent UWB range dictionary
        self.agent_uwb_range_dic[f"{self.system_id}"] = uwb_pub_msg

    def timer_monitoring_pub_callback(self):
        self.monitoring_publisher.publish(self.monitoring_msg)

    ### Mission Progress ####
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
            self.share_target()

    ## Sub callback ##
    def monitoring_callback(self, msg: Monitoring):
        self.monitoring_msg = msg

    def uwb_msg_callback(self, msg: LinktrackNodeframe2):
        self.uwb_sub_msg = msg

    def global_path_callback(self, msg: GlobalPathMsg):
        self.get_logger().error("handle_global_path subscription called")
        self.global_path = [
            [pt.position[0], pt.position[1], pt.position[2]]
            for pt in msg.waypoints
        ]
        self.mission_zlevel = -msg.waypoints[0].position[2]

    def timestamp_callback(self, msg: Header):
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
        self.publish_target()
        # if self.particle is not None:
        #     self.publish_particle_cloud(self.particle)
        if self.mode_handler.is_in_mode(Mode.SEARCH):
            self.change_mode(Mode.HAVE_TARGET)

        if self.have_target == False:
            self.have_target = True
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
            for key in self.agent_target_dic:
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
        # err1, err2, err3 = self.f_formation.get_error(agents_pos)
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
        self.get_logger().info(f"DroneManager {self.system_id} : grad_norm : {grad_norm}")
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
        # direction = TrajectorySetpointMsg()
        # direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        # self.total_gradient_publisher.publish(direction)
        self.traj_setpoint_publisher.publish(setpoint)

    def is_formation_converged(self, grad_norm):
        # for value in self.agent_uwb_range_dic.values():
        #     if value.seq == Mode.RETURN:
        #         self.change_mode(Mode.RETURN)
        #         return
        if grad_norm < self.tol:
            self.change_mode(Mode.CONVERGED)
        # else:
        #     if self.mode_handler.is_in_mode(Mode.CONVERGED):
        #         self.change_mode(Mode.HAVE_TARGET)
        if all(v.seq == Mode.CONVERGED.value for v in self.agent_uwb_range_dic.values()):
            self.get_logger().info(f"DroneManager {self.system_id} : Formation Converged")
            if self.system_id == self.system_id_list[0]:
                self.change_mode(Mode.COLLECTION, delay_seconds=1.0)
            else:
                self.change_mode(Mode.RETURN, delay_seconds=3.0)
            return
    
    def compute_weight(self):
        if self.have_target:
            current_pos = np.array([
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y
            ])
            target_pos = np.array([self.target[0], self.target[1]])
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

    ## Publisher ##
    def publish_target(self):
        target_msg = TrajectorySetpointMsg()
        target_pos_ned = [self.target[0], self.target[1], 0.1]
        ref_llh = [
            self.monitoring_msg.ref_lat,
            self.monitoring_msg.ref_lon,
            self.monitoring_msg.ref_alt
        ]
        target_pos_llh = NED2LLH(NED=target_pos_ned, ref_LLH=ref_llh)

        target_msg.position[0] = target_pos_llh[0]
        target_msg.position[1] = target_pos_llh[1]
        target_msg.position[2] = target_pos_llh[2]

        self.send_target_data(target_msg)
        # self.target_publisher.publish(target_msg)

    # def publish_particle_cloud(self, particles: np.ndarray):
    #     header = std_msgs.Header()
    #     header.frame_id = 'map'
    #     header.stamp    = self.get_clock().now().to_msg()

    #     points = [(float(x), float(y), 0.0) for x,y in particles]
    #     cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
    #     self.particle_publisher.publish(cloud_msg)
    
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
        # direction = TrajectorySetpointMsg()
        # direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        # self.total_gradient_publisher.publish(direction)
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
        grad_target = self.f_target.compute(
            current_pos=[
                self.monitoring_msg.pos_x,
                self.monitoring_msg.pos_y,
                self.monitoring_msg.pos_z,
                ],
            target=[
                self.target[0],
                self.target[1],
                - ( self.mission_zlevel + 2.0 ),
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
        # direction = TrajectorySetpointMsg()
        # direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        # self.total_gradient_publisher.publish(direction)
        self.traj_setpoint_publisher.publish(setpoint)

        if self.remain_distance(current_pos = [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y],
                                    target_pos = [self.target[0], self.target[1]]
                                    ) <= 0.5:
            self.get_logger().info(f"DroneManager {self.system_id} : Collection Completed")
            self.handle_flag = True
            self.change_mode(Mode.RETURN, delay_seconds=5.0)

    ## Utility ##
    def calculate_takeoff_offset(self):
        self.takeoff_offset_dic.clear()
        ref_LLH = [
            self.monitoring_msg.ref_lat,
            self.monitoring_msg.ref_lon,
            self.monitoring_msg.ref_alt
        ]
        for key, value in self.agent_uwb_range_dic.items():
            try:
                if value.anchor_pose.orientation.x == 0.0:
                    raise ValueError("Orientation.x is zero")
                LLH = [
                    value.anchor_pose.orientation.x,
                    value.anchor_pose.orientation.y,
                    value.anchor_pose.orientation.z
                ]
                if any(math.isnan(val) for val in LLH) or any(math.isnan(val) for val in ref_LLH):
                    raise ValueError("NaN in coordinates")
                NED = LLH2NED(LLH, ref_LLH)
                self.takeoff_offset_dic[f"{key}"] = NED
            except Exception as e:
                self.get_logger().warn(f"Key {key} skipped: {e}")

    def remain_distance(self, current_pos, target_pos):
        return math.sqrt(
            (current_pos[0] - target_pos[0])**2 +
            (current_pos[1] - target_pos[1])**2
        )

    def change_mode(self, mode: Mode):
        self.mode = mode
        self.get_logger().info(f"Mode Changed : {mode}")
        if mode == Mode.QHAC:
            self.change_ocm_msg_position()
        elif mode == Mode.HAVE_TARGET:
            pass
    
    def change_mode(self, mode, delay_seconds= None):
        if delay_seconds is None:
            result = self.mode_handler.change_mode(mode)
            if result == -1:
                self.get_logger().warn(f"DroneManager {self.system_id} : Mode Change Failed to {Mode(mode)}")
            else:
                self.get_logger().info(f"DroneManager {self.system_id} : Mode Changed to {Mode(mode)}")
        else:
            result = self.mode_handler.change_mode_delay(mode, delay_seconds)
            if result == -1:
                self.get_logger().warn(f"DroneManager {self.system_id} : Mode Change Delay Failed to {Mode(mode)}")
            else:
                self.get_logger().info(f"DroneManager {self.system_id} : Mode Change Delay Start... {Mode(mode)} | {delay_seconds} seconds")

    ## OCM Msg ##
    def change_ocm_msg_position(self):
        self.ocm_msg.position     = True
        self.ocm_msg.velocity     = False
        self.ocm_msg.acceleration = False
        self.ocm_msg.attitude     = False
        self.ocm_msg.body_rate    = False
        self.ocm_msg.actuator     = False
        self.get_logger().info(f"DroneManager {self.system_id} : OCM changed : Position")

    def change_ocm_msg_velocity(self):
        self.ocm_msg.position     = False
        self.ocm_msg.velocity     = True
        self.ocm_msg.acceleration = False
        self.ocm_msg.attitude     = False
        self.ocm_msg.body_rate    = False
        self.ocm_msg.actuator     = False
        self.get_logger().info(f"DroneManager {self.system_id} : OCM changed : Velocity")

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
    node = DroneManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
