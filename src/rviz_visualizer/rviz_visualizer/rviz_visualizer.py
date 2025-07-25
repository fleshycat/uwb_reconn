import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import SuvMonitoring, LogMessage, Monitoring, VehicleStatus, OffboardControlMode, TrajectorySetpoint as TrajectorySetpointMsg, VehicleCommandAck, VehicleCommand as VehicleCommandMsg, DistanceSensor, GlobalPath as GlobalPathMsg
from uwb_msgs.msg import ParticleDistribution

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Point
from scipy.spatial.transform import Rotation as R
import numpy as np
import std_msgs.msg as std_msgs
from sensor_msgs.msg  import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import struct

class rivzVisualizer(Node):
    def __init__(self):
        super().__init__("rviz_visualizer")
        self.get_logger().info("Rviz visualizer Started")
        self.declare_parameter('system_id_list', [1,2,3,4])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        self.ref_agent_sys_id = self.system_id_list[0]

        self.declare_parameter('real_tag_pos', [0.0, 0.0])
        self.real_tag_pos = self.get_parameter('real_tag_pos').get_parameter_value().double_array_value
        self.declare_parameter('real_ref_agent_pos', [0.0, 0.0])
        self.real_ref_agent_pos = self.get_parameter('real_ref_agent_pos').get_parameter_value().double_array_value

        self.agents_target_dic = {f'{i}':TrajectorySetpointMsg() for i in self.system_id_list}
        self.agents_pos_dic = {f'{i}':Monitoring() for i in self.system_id_list}
        self.agents_particle_dic = {f'{i}':PointCloud2() for i in self.system_id_list}
        self.agents_gradient_dic = {f'{i}':TrajectorySetpointMsg() for i in self.system_id_list}
        self.agents_metrics_dic = {f'{i}':ParticleDistribution() for i in self.system_id_list}

        self.topic_prefix_rviz = "rviz/visualize"

        self.color_table = [[1.0, 0.0, 0.0, 1.0],
                            [0.0, 1.0, 0.0, 1.0],
                            [0.0, 0.0, 1.0, 1.0],
                            [0.0, 0.0, 0.0, 1.0]]
        ## Publisher ##
        self.targets_publisher = self.create_publisher(MarkerArray, f'{self.topic_prefix_rviz}/targets', qos_profile_sensor_data)
        self.agents_publisher = self.create_publisher(MarkerArray, f'{self.topic_prefix_rviz}/agents', qos_profile_sensor_data)
        self.real_tag_pos_publisher = self.create_publisher(Marker, f'{self.topic_prefix_rviz}/real_tag_pos', qos_profile_sensor_data)
        self.particle_publishers = [
            self.create_publisher(PointCloud2, f'{self.topic_prefix_rviz}/agent{i}/particle_cloud', qos_profile_sensor_data)
            for i in self.system_id_list
        ]
        self.grdients_publisher = self.create_publisher(MarkerArray, f'{self.topic_prefix_rviz}/gradients', qos_profile_sensor_data)
        self.metrics_publisher = self.create_publisher(MarkerArray, f'{self.topic_prefix_rviz}/particle_distribution', qos_profile_sensor_data)

        ## Subscriber ##
        self.agent_pos_subscribers = [
            self.create_subscription(Monitoring, f'drone{i}/fmu/out/monitoring', self.make_monitoring_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list
        ]
        self.target_subscribers = [
            self.create_subscription(TrajectorySetpointMsg, f'drone{i}/manager/out/target', self.make_target_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list
        ]
        self.particle_subscribers = [
            self.create_subscription(PointCloud2, f'drone{i}/manager/out/particle_cloud', self.make_particle_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list
        ]
        self.gradients_subscribers = [
            self.create_subscription(TrajectorySetpointMsg, f'drone{i}/manager/out/gradient', self.make_gradient_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list
        ]
        self.metrics_subscribers = [
            self.create_subscription(ParticleDistribution, f'drone{i}/manager/out/particle_distribution', self.make_metrics_callback(i), qos_profile_sensor_data)
            for i in self.system_id_list
        ]
        self.real_tag_pos_subscriber = self.create_subscription(Marker, f'uwb_tag_marker', self.make_tag_callback(), qos_profile_sensor_data)


        ## Timer ##
        timer_period_target = 0.1
        self.timer_ocm = self.create_timer(timer_period_target, self.publish_rviz_topic)

    def make_tag_callback(self):
        def callback(msg):
            self.real_tag_pos = [msg.pose.position.x, msg.pose.position.y]
            self.publish_tag()
        return callback
    
    def make_target_callback(self, sys_id):
        def callback(msg):
            self.agents_target_dic[f'{sys_id}'] = msg
        return callback
    
    def make_monitoring_callback(self, sys_id):
        def callback(msg):
            self.agents_pos_dic[f'{sys_id}'] = msg
        return callback

    def make_particle_callback(self, sys_id):
        def callback(msg):
            self.agents_particle_dic[f'{sys_id}'] = msg
        return callback
    
    def make_gradient_callback(self, sys_id):
        def callback(msg):
            self.agents_gradient_dic[f'{sys_id}'] = msg
        return callback

    def make_metrics_callback(self, sys_id):
        def callback(msg):
            self.agents_metrics_dic[f'{sys_id}'] = msg
        return callback

    def publish_rviz_topic(self):
        self.publish_target()
        self.publish_agents()
        self.publish_gradient()
        self.publish_particle_metrics()
        for i in self.system_id_list:
            self.publish_particle_cloud(i) 

    def publish_target(self):
        if len(self.agents_target_dic) == 0:
            return
        targets = MarkerArray()
        ref_llh = [
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lat,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lon,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_alt,
            ]
        for key, value in self.agents_target_dic.items():
            target = Marker()
            target.header.frame_id = 'map'
            target.header.stamp    = self.get_clock().now().to_msg()
            target.ns    = 'target'
            target.id    = int(key)
            target.type  = Marker.SPHERE    # 또는 Marker.CUBE
            target.action= Marker.ADD
            target.scale.x = 0.2
            target.scale.y = 0.2
            target.scale.z = 0.2
            target.color.r = self.color_table[int(key)-1][0]
            target.color.g = self.color_table[int(key)-1][1]
            target.color.b = self.color_table[int(key)-1][2]
            target.color.a = self.color_table[int(key)-1][3] / 1.0
            target_pos_llh = [
                float(value.position[0]),
                float(value.position[1]),
                0.1
            ]
            target_pos_ned = LLH2NED(target_pos_llh, ref_llh)
            target.pose.position.x = float(target_pos_ned[1])
            target.pose.position.y = float(target_pos_ned[0])
            target.pose.position.z = 0.1
            target.pose.orientation.w = 1.0
            targets.markers.append(target)

        self.targets_publisher.publish(targets)
    
    def publish_agents(self):
        if len(self.agents_pos_dic) == 0:
            return
        agents = MarkerArray()
        ref_llh = [
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lat,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lon,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_alt,
            ]
        stamp = self.get_clock().now().to_msg()
        for key, value in self.agents_pos_dic.items():
            agent_pos = Marker()
            agent_pos.header.frame_id = 'map'
            agent_pos.header.stamp = stamp
            agent_pos.ns = 'agents'
            agent_pos.id = int(key)
            agent_pos.type = Marker.ARROW
            agent_pos.action = Marker.ADD
            agent_pos_llh = [
                value.lat,
                value.lon,
                value.alt
            ]
            agent_pos_ned = LLH2NED(agent_pos_llh, ref_llh)
            agent_pos.pose.position.x = agent_pos_ned[1]
            agent_pos.pose.position.y = agent_pos_ned[0]
            agent_pos.pose.position.z = - value.pos_z
            r_ned = R.from_euler('xyz', [value.roll, value.pitch, value.head])
            offset = R.from_euler('z', -np.pi/2, degrees=False)
            q = (r_ned * offset).as_quat()
            agent_pos.pose.orientation.x = q[1]
            agent_pos.pose.orientation.y = q[0]
            agent_pos.pose.orientation.z = -q[2]
            agent_pos.pose.orientation.w = q[3]
            agent_pos.scale.x = 0.6
            agent_pos.scale.y = 0.1
            agent_pos.scale.z = 0.1
            agent_pos.color.r = self.color_table[int(key)-1][0]
            agent_pos.color.g = self.color_table[int(key)-1][1]
            agent_pos.color.b = self.color_table[int(key)-1][2]
            agent_pos.color.a = self.color_table[int(key)-1][3]
            agents.markers.append(agent_pos)
        self.agents_publisher.publish(agents)

    def publish_tag(self):
        tag = Marker()
        tag.header.frame_id = 'map'
        tag.header.stamp    = self.get_clock().now().to_msg()
        tag.ns    = 'tag'
        tag.id    = 0
        tag.type  = Marker.CUBE    # 또는 Marker.CUBE
        tag.action= Marker.ADD
        tag.scale.x = 0.4
        tag.scale.y = 0.4
        tag.scale.z = 0.4
        tag.color.r = 0.5
        tag.color.g = 0.4
        tag.color.b = 0.6
        tag.color.a = 0.7
        tag.pose.position.x = self.real_tag_pos[0] - self.real_ref_agent_pos[0]
        tag.pose.position.y = self.real_tag_pos[1] - self.real_ref_agent_pos[1]
        tag.pose.position.z = 0.1
        tag.pose.orientation.w = 1.0
        self.real_tag_pos_publisher.publish(tag)

    def publish_gradient(self):
        if len(self.agents_gradient_dic) == 0:
            return
        gradients = MarkerArray()
        ref_llh = [
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lat,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lon,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_alt,
            ]
        stamp = self.get_clock().now().to_msg()
        for key, value in self.agents_gradient_dic.items():
            gradient = Marker()
            gradient.header.frame_id = 'map'
            gradient.header.stamp = stamp
            gradient.ns = 'gradient'
            gradient.id = int(key)
            gradient.type = Marker.ARROW
            gradient.action = Marker.ADD
            agent_pos_llh = [
                self.agents_pos_dic[f'{key}'].lat,
                self.agents_pos_dic[f'{key}'].lon,
                self.agents_pos_dic[f'{key}'].alt,
            ]
            agent_pos_ned = LLH2NED(agent_pos_llh, ref_llh)
            
            start = Point()
            start.x = agent_pos_ned[1]
            start.y = agent_pos_ned[0]
            start.z = -self.agents_pos_dic[key].pos_z

            v = np.array([value.velocity[1], value.velocity[0], -value.velocity[2]])
            d = v / np.linalg.norm(v) 

            length = 1.5
            end = Point()
            end.x = start.x + d[0] * length
            end.y = start.y + d[1] * length
            end.z = start.z + d[2] * length
            gradient.scale.x = 0.05
            gradient.scale.y = 0.05
            gradient.scale.z = 0.0
            gradient.points = [start, end]
            gradient.color.r = 1.0
            gradient.color.g = 0.0
            gradient.color.b = 0.0
            gradient.color.a = 1.0
            gradients.markers.append(gradient)
        self.grdients_publisher.publish(gradients)
            
    def publish_particle_metrics(self):
        if len(self.agents_metrics_dic) == 0:
            return

        metrics_markers = MarkerArray()
        ref_llh = [
            self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lat,
            self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lon,
            self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_alt,
        ]
        stamp = self.get_clock().now().to_msg()
        
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        metrics_markers.markers.insert(0, delete_all)
        
        marker_id = 0
        for key, metrics in self.agents_metrics_dic.items():
            sys_id = int(key)
            
            # Get agent position
            agent_pos_llh = [
                self.agents_pos_dic[f'{key}'].lat,
                self.agents_pos_dic[f'{key}'].lon,
                self.agents_pos_dic[f'{key}'].alt,
            ]
            agent_pos_ned = LLH2NED(agent_pos_llh, ref_llh)
            sensor_pos_x = agent_pos_ned[1]
            sensor_pos_y = agent_pos_ned[0]
            
            # 1. Draw average radius circle
            radius_marker = Marker()
            radius_marker.header.frame_id = 'map'
            radius_marker.header.stamp = stamp
            radius_marker.ns = 'particle_radius'
            radius_marker.id = marker_id
            marker_id += 1
            radius_marker.type = Marker.CYLINDER
            radius_marker.action = Marker.ADD
            
            radius_marker.pose.position.x = sensor_pos_x
            radius_marker.pose.position.y = sensor_pos_y
            radius_marker.pose.position.z = 0.05
            radius_marker.pose.orientation.w = 1.0
            
            # Scale: diameter for x,y and height for z
            radius_marker.scale.x = max(0.01, metrics.avg_radius * 2.0)
            radius_marker.scale.y = max(0.01, metrics.avg_radius * 2.0)
            radius_marker.scale.z = 0.1
            
            # Color: translucent version of agent color
            rgba = self.color_table[sys_id - 1]
            radius_marker.color.r = rgba[0]
            radius_marker.color.g = rgba[1]
            radius_marker.color.b = rgba[2]
            radius_marker.color.a = 0.2  # translucent
            
            #metrics_markers.markers.append(radius_marker)
            
            # 2. Draw radius uncertainty range (min/max circles)
            radius_range = metrics.radius_std * 2.0  # ±2σ range
            
            # Min radius circle
            min_radius = max(0.1, metrics.avg_radius - radius_range)
            min_marker = Marker()
            min_marker.header.frame_id = 'map'
            min_marker.header.stamp = stamp
            min_marker.ns = 'particle_radius_range'
            min_marker.id = marker_id
            marker_id += 1
            min_marker.type = Marker.LINE_STRIP
            min_marker.action = Marker.ADD
            
            # Draw circle with line strip
            circle_points = []
            for i in range(101):  # 100 segments + close the circle
                angle = 2 * np.pi * i / 100
                point = Point()
                point.x = sensor_pos_x + min_radius * np.sin(angle)
                point.y = sensor_pos_y + min_radius * np.cos(angle)
                point.z = 0.1
                circle_points.append(point)
            
            min_marker.points = circle_points
            min_marker.scale.x = 0.02
            min_marker.color.r = rgba[0]
            min_marker.color.g = rgba[1]
            min_marker.color.b = rgba[2]
            min_marker.color.a = 0.5
            
            metrics_markers.markers.append(min_marker)
            
            # Max radius circle
            max_radius = metrics.avg_radius + radius_range
            max_marker = Marker()
            max_marker.header.frame_id = 'map'
            max_marker.header.stamp = stamp
            max_marker.ns = 'particle_radius_range'
            max_marker.id = marker_id
            marker_id += 1
            max_marker.type = Marker.LINE_STRIP
            max_marker.action = Marker.ADD
            
            # Draw circle with line strip
            circle_points = []
            for i in range(101):
                angle = 2 * np.pi * i / 100
                point = Point()
                point.x = sensor_pos_x + max_radius * np.sin(angle)
                point.y = sensor_pos_y + max_radius * np.cos(angle)
                point.z = 0.1
                circle_points.append(point)
            
            max_marker.points = circle_points
            max_marker.scale.x = 0.02
            max_marker.color.r = rgba[0]
            max_marker.color.g = rgba[1]
            max_marker.color.b = rgba[2]
            max_marker.color.a = 0.5
            
            metrics_markers.markers.append(max_marker)
            
            # 3. Draw bearing sector (angular range)
            
            sector_marker = Marker()
            sector_marker.header.frame_id = 'map'
            sector_marker.header.stamp = stamp
            sector_marker.ns = 'particle_bearing'
            sector_marker.id = marker_id
            marker_id += 1
            sector_marker.type = Marker.LINE_LIST
            sector_marker.action = Marker.ADD
            
            # Convert bearing to radians
            b_start = metrics.bearing_start
            b_end   = metrics.bearing_end
            
            span_deg = (b_end - b_start) % 360
            
            b_start_rad = np.radians(b_start)
            span_rad    = np.radians(span_deg)
            
            # Draw sector lines
            sector_lines = []
            vis_r = max(0.1, metrics.avg_radius)
            
            # Center to min bearing edge
            c = Point(x=sensor_pos_x, y=sensor_pos_y, z=0.15)
            e1 = Point(
                x=sensor_pos_x + vis_r * np.sin(b_start_rad),
                y=sensor_pos_y + vis_r * np.cos(b_start_rad),
                z=0.15
            )
            sector_lines.extend([c, e1])
            
            # Center to max bearing edge
            end_rad = b_start_rad + span_rad      # CHANGED
            e2 = Point(
                x=sensor_pos_x + vis_r * np.sin(end_rad),
                y=sensor_pos_y + vis_r * np.cos(end_rad),
                z=0.15
            )
            sector_lines.extend([c, e2])
            
            # Arc between min and max bearing
            arc_steps = 20
            for i in range(arc_steps):
                a1 = b_start_rad + span_rad * (i    ) / arc_steps
                a2 = b_start_rad + span_rad * (i + 1) / arc_steps
                p1 = Point(
                    x=sensor_pos_x + vis_r * np.sin(a1),
                    y=sensor_pos_y + vis_r * np.cos(a1),
                    z=0.15
                )
                p2 = Point(
                    x=sensor_pos_x + vis_r * np.sin(a2),
                    y=sensor_pos_y + vis_r * np.cos(a2),
                    z=0.15
                )
                
                sector_lines.extend([p1, p2])
                
                sector_marker.points = sector_lines
                sector_marker.scale.x = 0.05
                sector_marker.color.r = rgba[0]
                sector_marker.color.g = rgba[1]
                sector_marker.color.b = rgba[2]
                sector_marker.color.a = 0.8
                
                metrics_markers.markers.append(sector_marker)
        
        self.metrics_publisher.publish(metrics_markers)
            

    def publish_particle_cloud(self, sys_id):
        key = str(sys_id)
        if key not in self.agents_particle_dic:
            return
        msg = self.agents_particle_dic[key]
        if not msg.fields:
            return
        pts = point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True) 
        raw_points = list(pts)

        rgba = self.color_table[sys_id -1]
        r = int(rgba[0]*255); g = int(rgba[1]*255)
        b = int(rgba[2]*255); a = int(rgba[3]*255)
        packed = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

        ref_llh = [
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lat,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_lon,
                self.agents_pos_dic[f'{self.ref_agent_sys_id}'].ref_alt,
            ]
        agent_pos = [
            self.agents_pos_dic[f'{sys_id}'].ref_lat, 
            self.agents_pos_dic[f'{sys_id}'].ref_lon, 
            self.agents_pos_dic[f'{sys_id}'].ref_alt
            ]
        agent_pos_ned = LLH2NED(agent_pos, ref_llh)
        colored_pts = []
        for x, y, z in raw_points:
            nx = y + agent_pos_ned[1]
            ny = x + agent_pos_ned[0]
            nz = z
            colored_pts.append((nx, ny, nz, packed))
        
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
        ]
        header = std_msgs.Header()
        header.frame_id = 'map'
        header.stamp    = self.get_clock().now().to_msg()

        cloud_msg = point_cloud2.create_cloud(header, fields, colored_pts)
        self.particle_publishers[sys_id - 1].publish(cloud_msg)


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

    rviz_visualizer = rivzVisualizer()

    rclpy.spin(rviz_visualizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rviz_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()