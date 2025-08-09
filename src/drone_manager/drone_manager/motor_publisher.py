#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors, EscStatus
from rclpy.qos import qos_profile_sensor_data
import time

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_publisher')
        
        # PX4Swarm 모터 출력 토픽 발행
        self.actuator_motors_publisher = self.create_publisher(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            qos_profile_sensor_data
        )
        
        # PX4Swarm ESC 상태 토픽 발행
        self.esc_status_publisher = self.create_publisher(
            EscStatus,
            '/fmu/out/esc_status',
            qos_profile_sensor_data
        )
        
        # 타이머 설정 (1초마다 발행)
        self.timer = self.create_timer(1.0, self.publish_motor_data)
        
        self.get_logger().info('Motor Publisher Node Started')
        self.get_logger().info('Publishing to /fmu/out/actuator_motors')
        self.get_logger().info('Publishing to /fmu/out/esc_status')
    
    def publish_motor_data(self):
        """모터 데이터 발행"""
        # ActuatorMotors 메시지 생성
        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = self.get_clock().now().nanoseconds
        actuator_msg.reversible_flags = 0
        actuator_msg.control = [1000.0, 1000.0, 1000.0, 1000.0]  # 4개 모터 PWM 값
        
        # EscStatus 메시지 생성
        esc_msg = EscStatus()
        esc_msg.timestamp = self.get_clock().now().nanoseconds
        esc_msg.esc_count = 4
        esc_msg.esc_connectiontype = 0
        esc_msg.esc_online_flags = 15  # 모든 ESC 온라인
        esc_msg.esc_armed_flags = 15   # 모든 ESC 활성화
        esc_msg.esc_rpm = [2000, 2000, 2000, 2000]  # RPM 값
        esc_msg.esc_voltage = [12.0, 12.0, 12.0, 12.0]  # 전압
        esc_msg.esc_current = [1.0, 1.0, 1.0, 1.0]  # 전류
        esc_msg.esc_temperature = [25.0, 25.0, 25.0, 25.0]  # 온도
        
        # 메시지 발행
        self.actuator_motors_publisher.publish(actuator_msg)
        self.esc_status_publisher.publish(esc_msg)
        
        # self.get_logger().info(f'Published motor data: PWM={actuator_msg.control}, RPM={esc_msg.esc_rpm}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 