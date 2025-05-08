
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor
from nlink_parser_ros2_interfaces.msg import LinktrackNodeframe2
from uwb_msgs.msg import Ranging
from enum import Enum

from std_msgs.msg import Header

class Mode(Enum):
    pass

class DroneManager(Node):
    def __init__(self):
        super().__init__("drone_manager")
        self.declare_parameter('system_id', 1)
        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.get_logger().info(f"Configure DroneManager {self.system_id}")
        
        # self.mode
        
        self.topic_prefix_manager = f"drone{self.system_id}/manager/"  #"drone1/manager/"
        self.topic_prefix_fmu = f"drone{self.system_id}/fmu/"          #"drone1/fmu/"
        self.topic_prefix_uwb = f"drone{self.system_id}/uwb/ranging"

        self.monitoring_msg = Monitoring()
        self.uwb_sub_msg = LinktrackNodeframe2()
        self.uwb_pub_msg = Ranging()
        
        ## Publisher ##
        self.ocm_publisher = self.create_publisher(OffboardControlMode, f'{self.topic_prefix_fmu}in/offboard_control_mode', qos_profile_sensor_data)                    #"drone1/fmu/in/offboard_control_mode"
        self.uwb_ranging_publisher = self.create_publisher(Ranging, f'{self.topic_prefix_manager}out/ranging', qos_profile_sensor_data)
        
        ## Subscriber ##
        self.monitoring_subscriber = self.create_subscription(Monitoring, f'{self.topic_prefix_fmu}out/monitoring', self.monitoring_callback, qos_profile_sensor_data)  #"drone1/fmu/out/monitoring"
        self.uwb_subscriber = self.create_subscription(LinktrackNodeframe2, 'nlink_linktrack_nodeframe2', self.uwb_callback, qos_profile_sensor_data)
        self.timestamp_subscriber = self.create_subscription(Header, f'qhac/manager/in/timestamp',self.timestamp_callback, 10)

        self.ocm_msg = OffboardControlMode()
        self.ocm_msg.position = True
        self.ocm_msg.velocity = False
        self.ocm_msg.acceleration = False
        self.ocm_msg.attitude = False
        self.ocm_msg.body_rate = False
        self.ocm_msg.actuator = False
        
        self.gcs_timestamp = Header()
        self.init_timestamp = self.get_clock().now().to_msg().sec
        
        timer_period_ocm = 0.1
        self.timer_ocm = self.create_timer(timer_period_ocm, self.timer_ocm_callback)
        timer_period_uwb = 0.04
        self.timer_uwb = self.create_timer(timer_period_uwb, self.timer_uwb_callback)
        
    def timer_ocm_callback(self):
        self.ocm_publisher.publish(self.ocm_msg)
    
    def timestamp_callback(self, msg):
        self.get_logger().info("System Time Synchronize.")
        self.gcs_timestamp = msg
        
    def monitoring_callback(self, msg):
        self.monitoring_msg = msg

    def uwb_callback(self, msg):
        self.uwb_sub_msg = msg
        
    def timer_uwb_callback(self):
        self.uwb_pub_msg.header.frame_id            = "map"
        
        now_timestamp = self.get_clock().now().to_msg().sec
        self.uwb_pub_msg.header.stamp.sec = self.gcs_timestamp.stamp.sec + ( now_timestamp - self.init_timestamp )
        self.uwb_pub_msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        
        self.uwb_pub_msg.anchor_id                  = self.system_id
        
        # Only handle the distance of the tag
        # If there is no tag, the value is -1.0
        self.uwb_pub_msg.range = -1
        for node in self.uwb_sub_msg.nodes:
            if node.id == 0:
                self.uwb_pub_msg.range = int(node.dis * 1000)
                self.uwb_pub_msg.rss = node.rx_rssi
                self.uwb_pub_msg.error_estimation = node.fp_rssi
                break
        
        self.uwb_pub_msg.seq                        = self.uwb_sub_msg.system_time % 256
        self.uwb_pub_msg.anchor_pose.position.x     = self.monitoring_msg.pos_x
        self.uwb_pub_msg.anchor_pose.position.y     = self.monitoring_msg.pos_y
        self.uwb_pub_msg.anchor_pose.position.z     = self.monitoring_msg.pos_z
        self.uwb_pub_msg.anchor_pose.orientation.x  = self.monitoring_msg.ref_lat
        self.uwb_pub_msg.anchor_pose.orientation.y  = self.monitoring_msg.ref_lon
        self.uwb_pub_msg.anchor_pose.orientation.z  = self.monitoring_msg.ref_alt
        
        self.uwb_ranging_publisher.publish(self.uwb_pub_msg)
    
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