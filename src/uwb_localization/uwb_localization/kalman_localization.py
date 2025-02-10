
import rclpy
from rclpy.node import Node
from gtec_msgs.msg import Ranging, RangingList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from scipy.optimize import minimize
from tf2_ros import TransformBroadcaster

import numpy as np
import scipy.stats
from scipy.linalg import eigvals

# refer to https://soohwan-justin.tistory.com/47

class KalmanLocalizationNode(Node):
    def __init__(self):
        super().__init__('kalman_loc_node')

        self.mu = [0.0, 0.0, 0.0]
        self.sigma = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
        self.sensor_pos = {}
        self.pre_corrected_pos=[0.0, 0.0]
        self.pre_time_sec, self.pre_time_nanosec = self.get_clock().now().seconds_nanoseconds()
        
        self.declare_parameter('system_id', 1)
        self.system_id_ = self.get_parameter('system_id').get_parameter_value().integer_value

        self.declare_parameter('robot_type', 'iris')
        self.robot_type_ = self.get_parameter('robot_type').get_parameter_value().string_value
        
        self.topic_uwb_ = f"/gtec/toa/id_{self.system_id_}/ranging"
        # self.topic_imu_ = f"/{self.robot_type_}/id_{self.system_id_}/imu"
        self.topic_imu_ = "iris/id_down/imu"
        
        self.pub = self.create_publisher(PoseStamped, f'/{self.robot_type_}/id_{self.system_id_}/tag_kalman', 10)
        self.uwb_data_sub = self.create_subscription(
            RangingList,
            f'{self.topic_uwb_}',
            self.uwb_data_callback,
            10
        )

        self.imu_data_sub = self.create_subscription(
            Imu,
            f'{self.topic_imu_}',
            self.imu_data_callback,
            10
        )

        self.anchor_pos_sub = self.create_subscription(
            MarkerArray,
            f"/gtec/toa/id_{self.system_id_}/anchors",
            self.anchor_pos_callback,
            10
        )

    def anchor_pos_callback(self, maker_array):
        anchor_list = maker_array.markers
        for anchor in anchor_list:
            x, y, z = anchor.pose.position.x, anchor.pose.position.y, anchor.pose.position.z
            if anchor.id in self.sensor_pos :
                if self.sensor_pos[anchor.id] != [x,y,z] :
                    self.sensor_pos[anchor.id] = [x,y,z]
            else :
                self.sensor_pos[anchor.id] = [x,y,z]

    def imu_data_callback(self, imu_data):
        self.mu, self.sigma = self.prediction_step(imu_data)

    def uwb_data_callback(self, uwb_data):
        anchors = uwb_data.anchors
        sorted_anchors = sorted(anchors, key=lambda anchors: anchors.rss)

        count = 0
        for anchor in sorted_anchors[:4]:
            if anchor.anchor_id in self.sensor_pos:
                count += 1

        # self.get_logger().info(f'count : {count}')
        if count >= 2 :
            self.mu, self.sigma = self.correction_step(sorted_anchors[:4],  self.sensor_pos)

    def publish_data(self, pose_x, pose_y, pose_z):
        robot_pos = PoseStamped()

        robot_pos.header.stamp = self.get_clock().now().to_msg()
        robot_pos.header.frame_id = 'map'

        robot_pos.pose.position.x = float(pose_x)
        robot_pos.pose.position.y = float(pose_y)
        robot_pos.pose.position.z = float(pose_z)

        robot_pos.pose.orientation.x = 0.0
        robot_pos.pose.orientation.y = 0.0
        robot_pos.pose.orientation.z = 0.0
        robot_pos.pose.orientation.w = 1.0

        # self.get_logger().info(f"Publishing: {robot_pos.pose.position.x:.2f}, {robot_pos.pose.position.y:.2f}, {robot_pos.pose.position.z:.2f}")
        self.pub.publish(robot_pos)

    def prediction_step(self, imu_data):
        # Updates the belief, i.e., mu and sigma, according to the motion 
        # model
        # 
        # mu: 3x1 vector representing the mean (x,y,theta) of the 
        #     belief distribution
        # sigma: 3x3 covariance matrix of belief distribution 
        
        x = self.mu[0]
        y = self.mu[1]
        theta = self.mu[2]
        
        current_time_sec, current_time_nanosec = self.get_clock().now().seconds_nanoseconds()
        if current_time_sec == self.pre_time_sec:
            delta_t = (current_time_nanosec - self.pre_time_nanosec) * 10**-9
        else:
            delta_t = (current_time_nanosec - self.pre_time_nanosec) * 10**-9 + 1
        self.pre_time_sec, self.pre_time_nanosec = current_time_sec, current_time_nanosec
        
            
        delta_vel = np.sqrt(2 * imu_data.linear_acceleration.x**2 + imu_data.linear_acceleration.y**2) * delta_t
        #delta_vel = np.sqrt(imu_data.linear_acceleration.x**2 + imu_data.linear_acceleration.y**2) * delta_t
        
        delta_w = imu_data.angular_velocity.z * delta_t
        
        noise = 0.1**2
        v_noise = delta_vel**2
        w_noise = delta_w**2

        sigma_u = np.array([[noise + v_noise, 0.0],[0.0, noise + w_noise]])
        
        B = np.array([[np.cos(theta), 0.0],[np.sin(theta), 0.0],[0.0, 1.0]])

        #noise free motion
        x_new = x + delta_vel*np.cos(theta)
        y_new = y + delta_vel*np.sin(theta)
        theta_new = theta + delta_w
        
        #Jacobian of g with respect to the state
        G = np.array([[1.0, 0.0, -delta_vel * np.sin(theta)],
                        [0.0, 1.0, delta_vel * np.cos(theta)],
                        [0.0, 0.0, 1.0]])

        #new mu and sigma
        mu = [x_new, y_new, theta_new]
        sigma = np.dot(np.dot(G, self.sigma), np.transpose(G)) + np.dot(np.dot(B, sigma_u), np.transpose(B))

        #publish data every odom step
        self.publish_data(mu[0], mu[1], 0)

        return mu, sigma

        # updates the belief, i.e., mu and sigma, according to the

    def correction_step(self, uwb_data,  sensor_pos):
        # sensor model
        #
        # The employed sensor model is range-only
        #
        # mu: 3x1 vector representing the mean (x,y,theta) of the 
        #     belief distribution
        # sigma: 3x3 covariance matrix of belief distribution 

        x = self.mu[0]
        y = self.mu[1]
        z = 0
        theta = self.mu[2]

        #measured landmark ids and ranges
        ids = [uwb.anchor_id for uwb in uwb_data]
        ranges = [uwb.range/1000.0 for uwb in uwb_data]

        # Compute the expected range measurements for each landmark.
        # This corresponds to the function h
        H = []
        Z = []
        expected_ranges = []
        for i in range(len(ids)):
            id = ids[i]

            lx = sensor_pos[id][0]
            ly = sensor_pos[id][1]
            lz = sensor_pos[id][2]

            #calculate expected range measurement
            range_exp = np.sqrt( (lx - x)**2 + (ly - y)**2)
            # range_exp = np.sqrt( (lx - x)**2 + (ly - y)**2+(lz - z)**2 )

            #compute a row of H for each measurement
            H_i = [(x - lx)/range_exp, (y - ly)/range_exp, 0]
            H.append(H_i)
            Z.append(ranges[i])
            expected_ranges.append(range_exp)
            # self.get_logger().info(f'compare.. [{id}] {ranges[i]} {range_exp}')


        # noise covariance for the measurements
        R = 0.8 * np.eye(len(ids))

        # Kalman gain
        K_help = np.linalg.inv(np.dot(np.dot(H, self.sigma), np.transpose(H)) + R)
        K = np.dot(np.dot(self.sigma, np.transpose(H)), K_help)

        # Kalman correction of mean and covariance
        self.mu = self.mu + np.dot(K, (np.array(Z) - np.array(expected_ranges)))
        
        self.mu[2] = np.arctan2(self.mu[1] - self.pre_corrected_pos[1], self.mu[0] - self.pre_corrected_pos[0])
        self.pre_corrected_pos = [self.mu[0], self.mu[1]]
        
        self.sigma = np.dot(np.eye(len(self.sigma)) - np.dot(K, H), self.sigma)

        return self.mu, self.sigma

def main(args=None):
    rclpy.init(args=args)
    node = KalmanLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()