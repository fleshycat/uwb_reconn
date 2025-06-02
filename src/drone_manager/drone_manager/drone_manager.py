import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.srv import ModeChange, TrajectorySetpoint as TrajectorySetpointSrv, VehicleCommand as VehicleCommandSrv, GlobalPath as GlobalPathSrv
from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor, GlobalPath as GlobalPathMsg
from uwb_msgs.msg import Ranging
from nlink_parser_ros2_interfaces.msg import LinktrackNodeframe2

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

## J-Fi ##
import struct
import serial
import threading

class ParseState:
    WAIT_SYNC1 = 0
    WAIT_SYNC2 = 1
    WAIT_LENGTH = 2
    WAIT_HEADER = 3
    WAIT_DATA = 4
    WAIT_CHECKSUM_1 = 5
    WAIT_CHECKSUM_2 = 6

class JFiProtocol:
    HEADER_SIZE = 5  # JFiProtocol Header (SYNC1, SYNC2, length, seq, sid)
    CHECKSUM_SIZE = 2  # 16-bit checksum (Little Endian)

    def __init__(self):
        # Parser state
        self.state = ParseState.WAIT_SYNC1
        self.buffer = bytearray()
        self.checksum = 0
        self.payload_length = None

    @staticmethod
    def create_packet(payload, seq=1, sid=1):
        """
        Creates a JFiProtocol packet with a header, payload, and checksum.
        
        :param payload: Raw binary data (bytes)
        :param seq: Sequence number
        :param sid: Source ID
        :return: Complete packet as bytes
        """
        # SYNC bytes
        sync1 = 0xAA
        sync2 = 0x55

        # Compute total length (Header 5 bytes + Payload + Checksum 2 bytes)
        length = JFiProtocol.HEADER_SIZE + len(payload) + JFiProtocol.CHECKSUM_SIZE

        # Pack header (5 bytes): SYNC1, SYNC2, LENGTH, SEQ, SID
        header = struct.pack('BBBBB',
            sync1,      # 0: SYNC1 (0xAA)
            sync2,      # 1: SYNC2 (0x55)
            length,     # 2: LENGTH
            seq,        # 3: SEQ
            sid         # 4: SID
        )

        # Combine header and payload
        packet = header + payload

        # Compute 16-bit XOR checksum
        checksum = JFiProtocol.compute_checksum(packet)

        # Append checksum (2 bytes, Little Endian)
        packet += struct.pack('<H', checksum)

        return packet

    @staticmethod
    def compute_checksum(data):
        """
        Computes a 16-bit XOR checksum over the given data.

        :param data: Byte data for checksum computation
        :return: 16-bit checksum
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def reset_parser(self):
        self.state = ParseState.WAIT_SYNC1
        self.buffer = bytearray()
        self.checksum = 0

    def validate_checksum(self):
        # Compute checksum over all bytes except last two (which are the received checksum)
        if len(self.buffer) < JFiProtocol.CHECKSUM_SIZE:
            return False
        computed = 0
        for byte in self.buffer[:-JFiProtocol.CHECKSUM_SIZE]:
            computed ^= byte
        return computed == self.checksum
    
    def parse(self, byte):
        complete_packet = None

        if self.state == ParseState.WAIT_SYNC1:
            if byte == 0xAA:
                self.reset_parser()
                self.buffer.append(byte)
                self.state = ParseState.WAIT_SYNC2

        elif self.state == ParseState.WAIT_SYNC2:
            if byte == 0x55:
                self.buffer.append(byte)
                self.state = ParseState.WAIT_LENGTH
            else:
                self.reset_parser()

        elif self.state == ParseState.WAIT_LENGTH:
            self.buffer.append(byte)
            self.payload_length = byte
            self.state = ParseState.WAIT_HEADER

        elif self.state == ParseState.WAIT_HEADER:
            self.buffer.append(byte)
            if len(self.buffer) == JFiProtocol.HEADER_SIZE:
                self.state = ParseState.WAIT_DATA

        elif self.state == ParseState.WAIT_DATA:
            self.buffer.append(byte)
            data_length = self.payload_length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
            if len(self.buffer) == JFiProtocol.HEADER_SIZE + data_length:
                self.state = ParseState.WAIT_CHECKSUM_1

        elif self.state == ParseState.WAIT_CHECKSUM_1:
            self.buffer.append(byte)
            self.checksum = byte
            self.state = ParseState.WAIT_CHECKSUM_2

        elif self.state == ParseState.WAIT_CHECKSUM_2:
            self.buffer.append(byte)
            self.checksum |= (byte << 8)
            if self.validate_checksum():
                complete_packet = self.buffer[:]
            self.reset_parser()

        return complete_packet

class Mode(Enum):
    QHAC = 0
    SEARCH = 1
    HAVE_TARGET = 2

class DroneManager(Node):
    def __init__(self):
        super().__init__("drone_manager")
        self.declare_parameter('system_id', 1)
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.get_logger().info(f"Configure DroneManager {self.system_id}")
        self.declare_parameter('system_id_list', [1,2,3,4])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        
        ## J-Fi Configuration ##
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        serial_port_name = self.get_parameter('serial_port').value
        self.declare_parameter('baud_rate', 115200)
        baudrate = self.get_parameter('baud_rate').value
        self.jfi_seq = 0

        # Configure the serial port
        try:
            self.serial_port = serial.Serial(serial_port_name, baudrate=baudrate, timeout=1)
            self.get_logger().info(f"Opened serial port: {serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {serial_port_name}: {e}")
            return
    
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        ## J-Fi Configuration ##

        self.topic_prefix_manager = f"drone{self.system_id}/manager/"  #"drone1/manager/"
        self.topic_prefix_fmu = f"drone{self.system_id}/fmu/"          #"drone1/fmu/"
        self.topic_prefix_uwb = f"drone{self.system_id}/uwb/ranging"
        
        self.monitoring_msg = Monitoring()
        self.uwb_sub_msg = LinktrackNodeframe2()
        self.global_path = []
        self.global_path_threshold = 0.1
        self.takeoff_offset_dic = {}
        self.agent_uwb_range_dic = {f'{i}':Ranging() for i in self.system_id_list}
        self.agent_target_dic = {}
        self.mission_zlevel = 3.0
        self.direction = TrajectorySetpointMsg()

        ## Publisher ##
        self.ocm_publisher = self.create_publisher(OffboardControlMode, f'{self.topic_prefix_fmu}in/offboard_control_mode', qos_profile_sensor_data)                    #"drone1/fmu/in/offboard_control_mode"
        self.traj_setpoint_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_fmu}in/trajectory_setpoint', qos_profile_sensor_data)
        self.monitoring_publisher = self.create_publisher(Monitoring, f'{self.topic_prefix_manager}out/monitoring', qos_profile_sensor_data)
        # self.particle_publisher = self.create_publisher(PointCloud2, f'{self.topic_prefix_manager}out/particle_cloud', qos_profile_sensor_data)
        # self.total_gradient_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_manager}out/gradient', qos_profile_sensor_data)
        ## Optional
        self.uwb_ranging_publisher = self.create_publisher(Ranging, f'{self.topic_prefix_manager}out/ranging', qos_profile_sensor_data)
        self.target_publisher = self.create_publisher(TrajectorySetpointMsg, f'{self.topic_prefix_manager}out/target', qos_profile_sensor_data) 

        ## Subscriber ##
        self.uwb_subscriber = self.create_subscription(LinktrackNodeframe2, f'drone{self.system_id}/nlink_linktrack_nodeframe2', self.uwb_msg_callback, qos_profile_sensor_data)   # From UWB Sensor
        self.monitoring_subscriber = self.create_subscription(Monitoring, f'{self.topic_prefix_fmu}out/monitoring', self.monitoring_callback, qos_profile_sensor_data)  #"drone1/fmu/out/monitoring"
        self.timestamp_subscriber = self.create_subscription(Header, f'qhac/manager/in/timestamp',self.timestamp_callback, 10)
        self.global_path_subscriber = self.create_subscription(GlobalPathMsg, f'{self.topic_prefix_manager}in/global_path', self.global_path_callback, 10)
        ## Optional
        # self.agent_uwb_range_subscribers = [
        #     self.create_subscription(Ranging, f'drone{i}/manager/out/ranging', self.make_uwb_range_callback(i), qos_profile_sensor_data)
        #     for i in self.system_id_list if i != self.system_id
        # ]
        # self.agent_target_subscribers = [
        #     self.create_subscription(TrajectorySetpointMsg, f'drone{i}/manager/out/target', self.make_target_callback(i), qos_profile_sensor_data)
        #     for i in self.system_id_list if i != self.system_id
        # ]
        
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
        timer_period_ocm = 0.1          # 10hz
        self.timer_ocm = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        timer_period_uwb = 0.02         # 50hz
        self.timer_uwb = self.create_timer(timer_period_uwb, self.timer_uwb_callback)
        timer_period_global_path = 0.1  # 10hz
        self.timer_global_path = self.create_timer(timer_period_global_path, self.timer_global_path_callback)
        timer_period_mission = 0.04     # 25hz
        self.timer_mission = self.create_timer(timer_period_mission, self.timer_mission_callback)
        timer_period_monitoring = 0.02  # 50hz
        self.timer_monitoring = self.create_timer(timer_period_monitoring, self.timer_monitoring_pub_callback)
        
        self.gcs_timestamp = Header()
        self.init_timestamp = self.get_clock().now().to_msg().sec
        
        self.initiate_drone_manager()
    
    ## J-Fi Methods ##
    def send_uwb_data(self, uwb_pub_msg: Ranging):
        """
            Payload format for J-Fi ranging message:
          - B: anchor_id (uint8)
          - i: range_mm   (uint32)
          - B: seq_ranging (uint8)
          - d*3: pos_x, pos_y, pos_z (double*3)
          - d*3: ori_x, ori_y, ori_z (double*3)
          - f: rss (float32)
          - f: error_estimation (float32)
        """
        fmt = '<B i B d d d d d d f f'
        payload = struct.pack(
            fmt,
            uwb_pub_msg.anchor_id,                  # uint8
            uwb_pub_msg.range,                      # int32 (mm)
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
        packet = JFiProtocol.create_packet(payload, seq=self.jfi_seq, sid=self.system_id)

        # Send the packet via serial
        self.serial_port.write(packet)
        # Debug logging
        # self.get_logger().info(
        #     f"JFi TX → anchor_id={uwb_pub_msg.anchor_id}, "
        #     f"range_mm={uwb_pub_msg.range}, seq(jfi)={self.jfi_seq}"
        # )
    
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
        packet = JFiProtocol.create_packet(payload, seq=self.jfi_seq, sid=self.system_id)

        # Send the packet via serial
        self.serial_port.write(packet)
        # # Debug logging
        # self.get_logger().info(
        #     f"JFi TX → lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}, seq(jfi)={self.jfi_seq}"
        # )
    
    def receive_data(self):
        protocol_parser = JFiProtocol()
        while rclpy.ok():
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for b in data:
                        packet = protocol_parser.parse(b)

                        if packet is None:
                            continue

                        # Unpack header (5 bytes)
                        header = packet[:JFiProtocol.HEADER_SIZE]
                        sync1, sync2, length, seq_jfi, sid = struct.unpack('BBBBB', header)

                        # Unpack payload
                        payload_len = length - JFiProtocol.HEADER_SIZE - JFiProtocol.CHECKSUM_SIZE
                        payload = packet[JFiProtocol.HEADER_SIZE : JFiProtocol.HEADER_SIZE + payload_len]

                        # Unpack UWB payload data
                        if payload_len == 62:
                            anchor_id, \
                            range, \
                            seq_ranging, \
                            posx, posy, posz, \
                            orix, oriy, oriz, \
                            rss, \
                            error_est = struct.unpack('<B i B d d d d d d f f', payload)

                            # Recover Ranging
                            uwb_msg = Ranging()
                            uwb_msg.header.stamp = self.get_clock().now().to_msg()
                            uwb_msg.header.frame_id = "map"
                            uwb_msg.anchor_id = anchor_id
                            uwb_msg.range = range
                            uwb_msg.seq = seq_ranging
                            uwb_msg.anchor_pose.position.x = posx
                            uwb_msg.anchor_pose.position.y = posy
                            uwb_msg.anchor_pose.position.z = posz
                            uwb_msg.anchor_pose.orientation.x = orix
                            uwb_msg.anchor_pose.orientation.y = oriy
                            uwb_msg.anchor_pose.orientation.z = oriz
                            uwb_msg.rss = rss
                            uwb_msg.error_estimation = error_est

                            # Update the agent UWB range dictionary
                            self.agent_uwb_range_dic[f'{sid}'] = uwb_msg

                            # Log the received values
                            # self.get_logger().info(
                            #     f"JFi RX ← sid={sid}, anchor_id={anchor_id}, "
                            #     f"range={range}, rss={rss}, error_est={error_est}"
                            # )
                        
                        elif payload_len == 24:
                            lat, lon, alt = struct.unpack('<d d d', payload)

                            # Recover TrajectorySetpointMsg
                            target_msg = TrajectorySetpointMsg()
                            target_msg.position[0] = lat
                            target_msg.position[1] = lon
                            target_msg.position[2] = alt

                            # Update the agent target dictionary
                            self.agent_target_dic[f'{sid}'] = target_msg
                        
                        else:
                            # Ignore undefined payload length
                            self.get_logger().warn(
                                f"Undefined J-Fi payload length: {payload_len} bytes (SID={sid})"
                            )

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                break
    ## J-Fi Methods ##

    def initiate_drone_manager(self):
        self.change_mode(Mode.QHAC)
        self.agent_uwb_range_dic.clear()

    ## Timer callback ##
    def timer_ocm_callback(self):
        self.ocm_publisher.publish(self.ocm_msg)
    
    def timer_global_path_callback(self):
        if len(self.global_path) == 0 or self.mode != Mode.QHAC:
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
    
    # def timer_gradient_callback(self):
    #     if self.mode != Mode.HAVE_TARGET:
    #         return
    #     else:
    #         self.traj_setpoint_publisher.publish(self.direction)
    
    def timer_uwb_callback(self):
        # Create Ranging message
        uwb_pub_msg = Ranging()
        uwb_pub_msg.header.frame_id            = "map"

        # Set the timestamp for the message
        now_timestamp = self.get_clock().now().to_msg().sec
        uwb_pub_msg.header.stamp.sec = self.gcs_timestamp.stamp.sec + ( now_timestamp - self.init_timestamp )
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
                uwb_pub_msg.rss = node.rx_rssi
                uwb_pub_msg.error_estimation = node.fp_rssi
                break
        
        # If no node with ID 0 is found, return
        if uwb_pub_msg.range != -1:
            distance = uwb_pub_msg.range / 1000.0                       # mm → m
            height   = self.monitoring_msg.pos_z
            square_diff = max(distance**2 - height**2, 0)
            uwb_pub_msg.range   = int(math.sqrt(square_diff) * 1000)    # m → mm
        
        # UWB message sequence number
        uwb_pub_msg.seq     = self.uwb_sub_msg.system_time % 256
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
        self.agent_uwb_range_dic[f'{self.system_id}'] = uwb_pub_msg
    
    def timer_monitoring_pub_callback(self):
        self.monitoring_publisher.publish(self.monitoring_msg)

    ### Mission Progress ####
    def timer_mission_callback(self):
        if len(self.takeoff_offset_dic) != 4:
            self.calculate_takeoff_offset()
            return
        self.update_uwb_data_list()
        self.particle_step()
        self.move_agent()
        if len(self.uwb_data_list) >= 3:
            self.share_target()

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

    ## Particle Filter ##
    def particle_step(self):        
        if len(self.uwb_data_list) <= 0:
            # self.get_logger().info(f"DroneManager{self.system_id}: No uwb data available")
            self.have_target = False
            self.mode = Mode.QHAC
            # self.particle_filter.initialize([self.monitoring_msg.pos_x, self.monitoring_msg.pos_y])
            return
        
        sensor_positions = [row[0] for row in self.uwb_data_list]
        measurements = [row[1] for row in self.uwb_data_list]
        noise_stds = [row[2] for row in self.uwb_data_list]
        
        self.particle_filter.step(sensor_positions, measurements, noise_stds, [self.monitoring_msg.pos_x, self.monitoring_msg.pos_y])
        self.particle = self.particle_filter.particles
        
        self.target = self.particle_filter.estimate()
        estimate = [self.target[0], self.target[1]]
        self.publish_target(estimate)
        # if self.particle is not None:
            # self.publish_particle_cloud(self.particle)
        if self.have_target == False:
            self.have_target = True
            self.change_mode(Mode.HAVE_TARGET)
    
    def share_target(self):
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
    def move_agent(self):
        if not self.have_target:
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
        # direction = TrajectorySetpointMsg()
        # direction.velocity = [total_grad[0], total_grad[1], total_grad[2]]
        # self.total_gradient_publisher.publish(direction)
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

    ## Make and return callback
    # def make_uwb_range_callback(self, sys_id):
    #     self.get_logger().info(f"DroneManager {self.system_id} : Create Drone{sys_id} UWB Range Subscriber")
    #     def callback(msg):
    #         self.agent_uwb_range_dic[f'{sys_id}'] = msg
    #     return callback
    
    # def make_target_callback(self, sys_id):
    #     self.get_logger().info(f"DroneManager {self.system_id} : Create Drone{sys_id} Target Subscriber")
    #     def callback(msg):
    #         self.agent_target_dic[f'{sys_id}'] = msg
    #     return callback
    
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

        self.send_target_data(target)
        self.target_publisher.publish(target)

    # def publish_particle_cloud(self, particles: np.ndarray):
    #     header = std_msgs.Header()
    #     header.frame_id = 'map'
    #     header.stamp    = self.get_clock().now().to_msg()

    #     points = [(float(x), float(y), 0.0) for x,y in particles]
    #     cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
    #     self.particle_publisher.publish(cloud_msg)

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
                self.get_logger().warn(f"Key {key} skipped: {e}")

    def remain_distance(self, current_pos, target_pos):
        return math.sqrt((current_pos[0] - target_pos[0])**2 + (current_pos[1] - target_pos[1])**2)

    def change_mode(self, mode):
        self.mode = mode
        self.get_logger().info(f'Mode Changed : {mode}')
        if mode == Mode.QHAC:
            self.change_ocm_msg_position()
        elif mode == Mode.HAVE_TARGET:
            pass
            # self.change_ocm_msg_velocity()
        

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