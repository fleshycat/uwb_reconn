#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import Monitoring, TrajectorySetpoint
from geometry_msgs.msg import Vector3Stamped

# --- PI 컨트롤러 클래스 ---
class PIController:
    """간단한 PI 컨트롤러 클래스"""
    def __init__(self, kp, ki, integral_min, integral_max, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, error, dt):
        """
        PI 제어 값을 계산합니다.
        :param error: 현재 오차 (목표값 - 현재값)
        :param dt: 마지막 업데이트 이후 경과 시간
        :return: 제어 출력 값
        """
        # 비례항 (Proportional Term)
        p_term = self.kp * error

        # 적분항 (Integral Term) with Anti-Windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_min, self.integral_max)
        i_term = self.ki * self.integral

        # 최종 출력
        output = p_term + i_term

        # 출력 제한
        output = np.clip(output, self.output_min, self.output_max)

        return output

    def reset(self):
        """컨트롤러의 적분항을 리셋합니다."""
        self.integral = 0.0

class GimbalControllerNode(Node):
    def __init__(self, system_id: int = 1):
        super().__init__('px4_gimbal_controller')

        self.declare_parameter('system_id', 1)
        self.declare_parameter('monitoring_topic', '/fmu/out/monitoring')
        self.declare_parameter('trajectory_topic', '/jfi/in/target')
        self.declare_parameter('gimbal_controller_set_topic', f"/a8_mini/set_gimbal_attitude")
        self.declare_parameter('gimbal_controller_get_topic', f"/a8_mini/get_gimbal_attitude")

        self.system_id = self.get_parameter('system_id').get_parameter_value().integer_value
        self.monitoring_topic = self.get_parameter('monitoring_topic').get_parameter_value().string_value
        self.trajectory_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.gimbal_controller_set_topic = self.get_parameter('gimbal_controller_set_topic').get_parameter_value().string_value
        self.gimbal_controller_get_topic = self.get_parameter('gimbal_controller_get_topic').get_parameter_value().string_value

        # --- PI 컨트롤러 초기화 ---
        # Kp: 오차에 얼마나 빠르게 반응할지 결정
        # Ki: 정상상태 오차(steady-state error)를 없애기 위해 사용
        self.pi_pitch = PIController(kp=0.7, ki=0.15, integral_min=-5.0, integral_max=5.0, output_min=-90.0, output_max=45.0)
        self.pi_yaw = PIController(kp=0.7, ki=0.15, integral_min=-5.0, integral_max=5.0, output_min=-120.0, output_max=120.0)

        self.last_time = None # 마지막 콜백 실행 시간 저장

        # Global 변수 설정
        self.topic_prefix = f"/drone{self.system_id}"
        self.monitoring_msg = Monitoring()
        self.trajectory_msg = TrajectorySetpoint()
        self.gimbal_prev_msg = Vector3Stamped()

        # Subscriber 설정
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # 기존의 queue size와 동일하게 설정
            durability=DurabilityPolicy.VOLATILE
        )

        self.monitoring_sub = self.create_subscription(
            Monitoring,
            self.topic_prefix+self.monitoring_topic,  # 실제 모니터링 토픽 이름 맞춰 변경
            self.monitoring_callback,
            sensor_qos_profile
        )

        self.trajectory_sub = self.create_subscription(
            TrajectorySetpoint,
            self.topic_prefix+self.trajectory_topic,  # 실제 경로 토픽 이름 맞춰 변경
            self.trajectory_callback,
            sensor_qos_profile
        )

        self.gimbal_controller_get_topic = self.create_subscription(
            Vector3Stamped,
            self.topic_prefix+self.gimbal_controller_get_topic,  # 실제 짐벌 상태 토픽 이름 맞춰 변경
            self.gimbal_control_callback,
            sensor_qos_profile
        )
        # Publisher 설정
        topic_name = f"drone{self.system_id}a8_mini/set_gimbal_attitude"
        self.gimbal_pub = self.create_publisher(Vector3Stamped, self.topic_prefix+self.gimbal_controller_set_topic, 10)

        self.get_logger().info(f"Node started. Publishing gimbal commands to: {topic_name}")

    def monitoring_callback(self, msg: Monitoring):
        self.monitoring_msg = msg
        # self.get_logger().info(f"[Monitoring] Example value: {msg.timestamp}")

    def gimbal_control_callback(self, msg: Vector3Stamped):
        self.gimbal_prev_msg = msg
        # self.get_logger().info(f"[Gimbal Control] Received gimbal attitude: roll={msg.vector.x:.2f}, pitch={msg.vector.y:.2f}, yaw={msg.vector.z:.2f}")

    def trajectory_callback(self, msg: TrajectorySetpoint):
        target_position = []
        if not self.monitoring_msg.lat or not self.monitoring_msg.lon or not self.monitoring_msg.alt:
            self.get_logger().warn("Drone Position not initialized. Waiting for positoin data.")
            return

        # --- 시간 변화량 (dt) 계산 ---
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        target_position = LLH2NED(
            [msg.position[0], msg.position[1], msg.position[2]],
            [self.monitoring_msg.lat, self.monitoring_msg.lon, self.monitoring_msg.alt]
        )

        target_pose = NED2RPY(target_position, [0, 0, 0])
        target_msg = Vector3Stamped()

        target_msg.vector.x = target_pose[0] - self.monitoring_msg.roll  # roll
        target_msg.vector.y = -(target_pose[1] - self.monitoring_msg.pitch)  # pitch
        target_msg.vector.z = target_pose[2] - self.monitoring_msg.head  # yaw

        if not self.gimbal_prev_msg.vector.x and not self.gimbal_prev_msg.vector.y and not self.gimbal_prev_msg.vector.z:
            self.get_logger().warn("Gimbal attitude not initialized. Waiting for gimbal data.")
            return

        error_roll = 0.0  # Roll은 정의되지 않으므로 0으로 설정
        error_pitch = target_msg.vector.y - GimbalPitchGet2Set(self.gimbal_prev_msg.vector.y)
        error_yaw = target_msg.vector.z - self.gimbal_prev_msg.vector.z

        roll_cmd = error_roll
        pitch_cmd = self.pi_pitch.update(error_pitch, dt)
        yaw_cmd = self.pi_yaw.update(error_yaw, dt)

        gimbal_msg = Vector3Stamped()
        gimbal_msg.vector.x = roll_cmd  # Roll은 정의되지 않으므로 0으로 설정
        gimbal_msg.vector.y = pitch_cmd  # Pitch 명령
        gimbal_msg.vector.z = yaw_cmd  # Yaw 명령

        self.gimbal_pub.publish(gimbal_msg)
        self.get_logger().info(
            f"[Gimbal Publish] roll={gimbal_msg.vector.x:.2f}, pitch={gimbal_msg.vector.y:.2f}, yaw={gimbal_msg.vector.z:.2f}"
        )

def GimbalPitchGet2Set(desired_angle: float) -> float:

    # 1. 180도를 빼서 짐벌의 좌표계로 변환합니다.
    command_angle = desired_angle - 180.0

    # 2. 각도를 -180 ~ +180 범위로 정규화(Normalization)합니다.
    while command_angle > 180.0:
        command_angle -= 360.0
    while command_angle <= -180.0:
        command_angle += 360.0

    return command_angle

# WGS-84
a = 6378137.0
f = 1.0 / 298.257223563
e2 = 2 * f - f * f

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

def NED2RPY(NED, ref_NED):
    # 1. 기준 위치에서 목표 위치를 향하는 방향 벡터(Line of Sight) 계산
    los_vector = np.asarray(NED) - np.asarray(ref_NED)

    # 벡터의 각 성분 추출
    delta_n = los_vector[0]  # North 차이
    delta_e = los_vector[1]  # East 차이
    delta_d = los_vector[2]  # Down 차이

    # 2. Yaw (ψ) 계산
    # 북쪽(North)을 기준으로 동쪽(East) 방향으로의 각도를 계산
    # arctan2를 사용하여 올바른 사분면의 각도를 얻음
    yaw_rad = np.arctan2(delta_e, delta_n)

    # 3. Pitch (θ) 계산
    # 수평 거리(N-E 평면) 계산
    horizontal_distance = np.sqrt(delta_n**2 + delta_e**2)
    # 수평 거리에 대한 수직(Down) 방향의 각도를 계산
    pitch_rad = -np.arctan2(delta_d, horizontal_distance)

    # 4. Roll (φ)은 정의되지 않으므로 0으로 가정
    roll_rad = 0.0
    return [np.rad2deg(roll_rad), np.rad2deg(pitch_rad), np.rad2deg(yaw_rad)]

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode(system_id=1)  # system_id 변경 가능
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
