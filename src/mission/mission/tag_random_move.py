#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import numpy as np

class RandomMover(Node):
    def __init__(self):
        super().__init__('random_mover')
        # Parameter for reference agent offset
        self.declare_parameter('real_ref_agent_pos', [0.0, 0.0])
        self.real_ref_agent_pos = self.get_parameter('real_ref_agent_pos').value

        # Service client for setting entity state
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /set_entity_state service...')

        # Model name and movement bounds
        self.model_name = 'uwb_tag1'
        self.x_min, self.x_max = -10.0, 10.0
        self.y_min, self.y_max = 0.0, 15.0
        self.speed = 2.0

        # Random initial velocity vector
        angle = np.random.uniform(0, 2 * np.pi)
        self.velocity = np.array([np.cos(angle), np.sin(angle), 0.0]) * self.speed

        # Timer to move model
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Subscribe to Gazebo model states
        self.sub = self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)
        # Publisher for RViz marker
        self.marker_pub = self.create_publisher(Marker, 'uwb_tag_marker', 10)
        # TF broadcaster placeholder (optional)
        self.tf_broadcaster = TransformBroadcaster(self)

    def timer_callback(self):
        # Initialize position
        if not hasattr(self, 'position'):
            self.position = np.array([
                np.random.uniform(self.x_min, self.x_max),
                np.random.uniform(self.y_min, self.y_max),
                0.1
            ])

        # Update position by velocity
        dt = 0.1
        self.position[:2] += self.velocity[:2] * dt
        # Reflect on boundaries
        if self.position[0] < self.x_min or self.position[0] > self.x_max:
            self.velocity[0] *= -1
        if self.position[1] < self.y_min or self.position[1] > self.y_max:
            self.velocity[1] *= -1

        # Send SetEntityState request
        req = SetEntityState.Request()
        state = EntityState()
        state.name = self.model_name
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = self.position
        state.pose = pose
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = self.velocity
        state.twist = tw
        req.state = state
        self.client.call_async(req)

    def model_states_callback(self, msg: ModelStates):
        # Find index of our model
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return

        # Get pose and apply reference offset
        p = msg.pose[idx].position
        x = p.x
        y = p.y
        z = p.z

        # Create and publish a sphere marker at the model position
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'uwb_tag'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RandomMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
