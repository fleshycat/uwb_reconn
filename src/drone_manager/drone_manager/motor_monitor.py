#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors, EscStatus
from rclpy.qos import qos_profile_sensor_data
import time

class MotorMonitor(Node):
    def __init__(self):
        super().__init__('motor_monitor')
        
        # 드론 ID 리스트
        self.drone_ids = [1, 2, 3, 4]
        
        # 로그 제어 변수
        self.last_log_time = {}  # 각 드론별 마지막 로그 시간
        self.log_interval = 10.0  # 10초마다 로그 출력
        self.last_motor_values = {}  # 이전 모터 값 저장
        
        # 각 드론의 모터 출력 토픽 구독
        self.actuator_motors_subscribers = {}
        self.esc_status_subscribers = {}
        
        for drone_id in self.drone_ids:
            # ActuatorMotors 구독
            self.actuator_motors_subscribers[drone_id] = self.create_subscription(
                ActuatorMotors,
                f'/drone{drone_id}/fmu/out/actuator_motors',
                lambda msg, d_id=drone_id: self.actuator_motors_callback(msg, d_id),
                qos_profile_sensor_data
            )
            
            # EscStatus 구독
            self.esc_status_subscribers[drone_id] = self.create_subscription(
                EscStatus,
                f'/drone{drone_id}/fmu/out/esc_status',
                lambda msg, d_id=drone_id: self.esc_status_callback(msg, d_id),
                qos_profile_sensor_data
            )
            
            # 초기화
            self.last_log_time[drone_id] = 0
            self.last_motor_values[drone_id] = None
        
        self.get_logger().info('Motor Monitor Node Started (Reduced logging mode)')
        self.get_logger().info(f'Subscribing to {len(self.drone_ids)} drones')
    
    def actuator_motors_callback(self, msg, drone_id):
        """모터 출력값 콜백 함수"""
        current_time = time.time()
        
        # 현재 모터 값
        current_values = msg.control[:4]  # 처음 4개 모터만
        
        # 로그 출력 조건:
        # 1. 마지막 로그로부터 10초가 지났거나
        # 2. 모터 값이 크게 변화했을 때 (중요한 변화 감지)
        should_log = False
        
        if current_time - self.last_log_time[drone_id] > self.log_interval:
            should_log = True
        elif self.last_motor_values[drone_id] is not None:
            # 모터 값 변화 감지 (임계값: 0.1)
            changes = [abs(current - last) for current, last in zip(current_values, self.last_motor_values[drone_id])]
            if any(change > 0.1 for change in changes):
                should_log = True
        
        if should_log:
            self.get_logger().info(f'Drone{drone_id} Motor Output: PWM={current_values}')
            self.last_log_time[drone_id] = current_time
        
        self.last_motor_values[drone_id] = current_values
    
    def esc_status_callback(self, msg, drone_id):
        """ESC 상태 콜백 함수"""
        # ESC 정보는 중요한 변화가 있을 때만 출력 (현재는 비활성화)
        # 필요시 주석 해제하여 사용
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 