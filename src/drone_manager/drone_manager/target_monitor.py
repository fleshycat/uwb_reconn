#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
import time
import numpy as np

# WGS-84 상수 (drone_manager.py와 동일)
a = 6378137.0
f = 1.0 / 298.257223563
e2 = 2 * f - f * f

def LLH2NED(LLH, ref_LLH):
    """LLH 좌표를 NED 좌표로 변환"""
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

def LLH2Gazebo(LLH, ref_LLH):
    """LLH 좌표를 Gazebo 좌표계로 변환 (ENU)"""
    NED = LLH2NED(LLH, ref_LLH)
    # NED를 ENU로 변환 (North->East, East->North, Down->Up)
    ENU = [NED[1], NED[0], -NED[2]]  # E, N, U
    return ENU

class TargetMonitor(Node):
    def __init__(self):
        super().__init__('target_monitor')
        
        # particle cloud만 구독 (target 토픽 제거)
        self.particle_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/drone1/manager/out/particle_cloud',
            self.particle_cloud_callback,
            qos_profile_sensor_data
        )
        
        # 새로운 토픽으로 publish (PointStamped 사용)
        self.target_publisher = self.create_publisher(
            PointStamped,
            '/drone1/target_estimate',
            10
        )
        
        # 상태 변수
        self.has_received_particles = False
        self.last_log_time = 0
        self.log_interval = 2.0  # 2초마다 로그 출력
        self.message_count = 0
        self.start_time = time.time()
        self.particles = None
        
        # Reference LLH (실제 PX4 HOME 좌표)
        self.ref_LLH = [36.6299, 127.4588, 0.0]  # PX4_HOME_LAT, PX4_HOME_LON
        
        self.get_logger().info('=== Target Monitor Node Started ===')
        self.get_logger().info('Subscribing to /drone1/manager/out/particle_cloud')
        self.get_logger().info('Publishing to /drone1/target_estimate (Gazebo ENU coordinates)')
        self.get_logger().info(f'Reference LLH: {self.ref_LLH}')
        self.get_logger().info(f'Start time: {time.strftime("%H:%M:%S", time.localtime(self.start_time))}')
        
        # 토픽 존재 여부 확인을 위한 타이머
        self.topic_check_timer = self.create_timer(5.0, self.check_topic_availability)
    
    def check_topic_availability(self):
        """토픽이 존재하는지 확인"""
        try:
            topics = self.get_topic_names_and_types()
            particle_topic = '/drone1/manager/out/particle_cloud'
            available_topics = [topic[0] for topic in topics]
            
            if particle_topic in available_topics:
                self.get_logger().info(f'✓ Particle cloud topic found: {particle_topic}')
            else:
                self.get_logger().warn(f'✗ Particle cloud topic not found: {particle_topic}')
                
        except Exception as e:
            self.get_logger().error(f'Error checking topic availability: {e}')
    
    def particle_cloud_callback(self, msg):
        """Particle cloud 콜백 함수 - 직접 estimate 계산"""
        try:
            # PointCloud2에서 particles 추출
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                # 각 point를 float로 변환
                x = float(point[0])
                y = float(point[1])
                z = float(point[2])
                points.append([x, y, z])
            
            if len(points) > 0:
                # points를 numpy 배열로 변환
                particles = np.array(points)
                
                # Weighted average로 estimate 계산
                estimate = np.mean(particles[:, :2], axis=0)  # x, y만 사용
                
                # Gazebo 좌표계로 변환 (particles는 이미 NED 좌표)
                gazebo_coords = [estimate[1], estimate[0], 0.0]  # NED -> ENU
                
                # PointStamped 메시지 생성
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = "map"
                point_msg.point.x = gazebo_coords[0]  # East (Gazebo X)
                point_msg.point.y = gazebo_coords[1]  # North (Gazebo Y)
                point_msg.point.z = gazebo_coords[2]  # Up (Gazebo Z)
                
                # 새로운 토픽으로 publish
                self.target_publisher.publish(point_msg)
                
                # 첫 번째 메시지 수신 시
                if not self.has_received_particles:
                    self.has_received_particles = True
                    elapsed_time = time.time() - self.start_time
                    self.get_logger().info('=== First Particle Estimate Received! ===')
                    self.get_logger().info(f'Timestamp: {time.strftime("%H:%M:%S", time.localtime(time.time()))}')
                    self.get_logger().info(f'Elapsed time: {elapsed_time:.2f}s')
                    self.get_logger().info(f'Particle estimate: NED=[{estimate[0]:.3f}, {estimate[1]:.3f}]')
                    self.get_logger().info(f'Gazebo ENU: X={gazebo_coords[0]:.3f}, Y={gazebo_coords[1]:.3f}, Z={gazebo_coords[2]:.3f}')
                    self.get_logger().info(f'Particles: {len(particles)}')
                
                # 주기적으로 로그 출력
                current_time = time.time()
                if current_time - self.last_log_time > self.log_interval:
                    timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                    self.get_logger().info(f'[{timestamp}] Particle-based estimate:')
                    self.get_logger().info(f'  Gazebo ENU: X={gazebo_coords[0]:.3f}, Y={gazebo_coords[1]:.3f}, Z={gazebo_coords[2]:.3f}')
                    self.last_log_time = current_time
                    
        except Exception as e:
            self.get_logger().error(f'Particle cloud processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 