#!/usr/bin/env python3
'''
__author__ = "Bekir Bostanci"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Bekir Bostanci"
__email__ = "bekirbostanci@gmail.com"
'''

import rclpy
from rclpy.node import Node
from uwb_msgs.msg import Ranging, RangingList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from scipy.optimize import minimize
from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, ApproximateTimeSynchronizer
from px4_msgs.msg import Monitoring
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from scipy.linalg import eigvals
import math

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_data_node')
        self.declare_parameter('system_id_list', [1,2,3,4])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        
        self.dictected_anchor_pos = np.zeros((len(self.system_id_list), 3))
        self.dictedted_anchor_ranging = np.zeros(len(self.system_id_list))
        
        self.TagPose_pub = self.create_publisher(PoseStamped, 'tag', qos_profile_sensor_data)
        self.TagPoseLLH_pub = self.create_publisher(Pose2D, 'tagLLH', qos_profile_sensor_data)
        
        self.uwb_topic_subscribers = [
            Subscriber(
                self, 
                Ranging, 
                f'drone{sys_id}/manager/out/ranging',
                qos_profile=qos_profile_sensor_data,
                ) 
            for sys_id in self.system_id_list
            ]

        self.topic_prefix_fmu_ = f"drone{self.system_id_list[0]}/fmu/"
        self.REF_drone_monitoring_subscriber = self.create_subscription(Monitoring, f'{self.topic_prefix_fmu_}out/monitoring', self.monitoring_callback, qos_profile_sensor_data)
        self.monitoring_msg = Monitoring()
        
        self.takeoff_offset = []
        self.takeoff_offset_flag = True
        self.ref_LLH = []
        
        self.ats = ApproximateTimeSynchronizer(
            self.uwb_topic_subscribers,
            queue_size=10,
            slop=0.1,
            )
        self.ats.registerCallback(self.uwb_data_callback)
            
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def uwb_data_callback(self, *msgs): 
        if self.takeoff_offset_flag:
            for i, msg in enumerate(msgs):
                if msg.anchor_pose.orientation.x == 0.0:
                    return
                if i==0:
                    ref_LLH = [msg.anchor_pose.orientation.x, msg.anchor_pose.orientation.y, msg.anchor_pose.orientation.z]
                LLH = [msg.anchor_pose.orientation.x, msg.anchor_pose.orientation.y, msg.anchor_pose.orientation.z]
                self.get_logger().info(f"ref_LLH : {ref_LLH}")
                self.get_logger().info(f"LLH : {LLH}")
                
                if any(math.isnan(val) for val in LLH) or any(math.isnan(val) for val in ref_LLH):
                    self.get_logger().warn("LLH ref_LLH NaN.")
                    return
                
                NED = LLH2NED(LLH, ref_LLH)
                self.takeoff_offset.append(NED)
            self.takeoff_offset_flag = False
            return
        
        for i, msg in enumerate(msgs):
            # anchorID = msg.anchor_id
            anchor_pose = np.array([msg.anchor_pose.position.x + self.takeoff_offset[i][0], 
                                    msg.anchor_pose.position.y + self.takeoff_offset[i][1], 
                                    msg.anchor_pose.position.z + self.takeoff_offset[i][2]])
            self.dictected_anchor_pos[i] = anchor_pose
            self.dictedted_anchor_ranging[i] = msg.range / 1000
        
        robot_pos=[]
        if len(self.dictected_anchor_pos)!=0:
            robot_pos = self.position_calculation(self.dictected_anchor_pos, self.dictedted_anchor_ranging)
        
        if robot_pos is not None and len(robot_pos) > 0:
            self.publish_data(robot_pos[0], robot_pos[1],robot_pos[2])
    
    def monitoring_callback(self, msg):
        self.monitoring_msg = msg
    
    def position_calculation(self, anchor_pos, dist):
        if len(anchor_pos) == 0:
            self.get_logger().warn("No anchor positions available.")
            return
        else:
            
            A=(-2*anchor_pos).transpose()
            vertical = len(anchor_pos[0])
            horizontal = len(anchor_pos)
            B=np.ones([horizontal,1],dtype=float).transpose()
            A=np.append(A,B,axis=0)
            A=A.transpose()
            R=np.zeros([1,vertical],dtype=float).transpose() 
            b=np.zeros([1,horizontal],dtype=float)
            for i in range(horizontal):
                b[0][i]=dist[i]**2-(np.linalg.norm(anchor_pos[i]))**2    
            D=np.eye(3)
            D=np.append(D,np.zeros([3,1],dtype=float),axis=1)
            D=np.append(D,np.zeros([1,4],dtype=float),axis=0)
            f=np.zeros([4,1],dtype=float)
            np.put(f,3,-0.5)
            AtrA=np.matmul(A.transpose(),A)
            eival=eigvals(D,AtrA)
            eival=np.real(np.sort(eival))
            eival=eival[::-1]
            lam=-(eival**-1)
            ub=10**7
            lb=lam[0]
            tolerance = 10**-4
            count=0
            while ub-lb>tolerance:
                count+=1
                midpo = (ub + lb)/2
                yhat=np.linalg.solve((AtrA+midpo*D),np.subtract(np.matmul(A.transpose(),b.transpose()).transpose(),(midpo*f).transpose()).transpose())
                #fun = yhat'*D*yhat + 2*f'*yhat;
                fun=np.matmul(np.matmul(yhat.transpose(),D),yhat)+np.matmul(2*f.transpose(),yhat)
                if fun>0:
                    lb = midpo
                else:
                    ub = midpo
            x=yhat[:3]
            if yhat[3]<0:
                x=-x
            return x 
        
    def publish_data(self, pose_x, pose_y, pose_z):        
        tag_pos = PoseStamped()
        tag_pos_LLH = Pose2D()
        
        tag_pos.header.stamp = self.get_clock().now().to_msg()
        tag_pos.header.frame_id = 'map'
        
        tag_pos.pose.position.x = float(pose_x)
        tag_pos.pose.position.y = float(pose_y)
        tag_pos.pose.position.z = float(pose_z)

        tag_pos.pose.orientation.x = 0.0
        tag_pos.pose.orientation.y = 0.0
        tag_pos.pose.orientation.z = 0.0
        tag_pos.pose.orientation.w = 1.0
        
        ref_LLH = [self.monitoring_msg.ref_lat, self.monitoring_msg.ref_lon, self.monitoring_msg.ref_alt]
        
        tag_pos_NED = [float(pose_x), float(pose_y), float(pose_z)]
        tag_pos_LLH_tmp = NED2LLH(NED=tag_pos_NED, ref_LLH=ref_LLH)
        
        # self.get_logger().info(f"tag_pos : {float(pose_x)}, {float(pose_y)}")
        # self.get_logger().info(f"tag_pos_NED : {tag_pos_NED}")
        # self.get_logger().info(f"tag_pos_LLH_tmp : {tag_pos_LLH_tmp}")
        # self.get_logger().info(f"anchor_LLH : {self.monitoring_msg.lat}, {self.monitoring_msg.lon}")
        # self.get_logger().info(f"ref_LLH : {ref_LLH}")
        # self.get_logger().info(f"self.REF_position : {self.REF_position}")
        
        tag_pos_LLH.x = tag_pos_LLH_tmp[0]
        tag_pos_LLH.y = tag_pos_LLH_tmp[1]
        tag_pos_LLH.theta = 0.0
        self.TagPose_pub.publish(tag_pos)
        self.TagPoseLLH_pub.publish(tag_pos_LLH)
            
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

    # 결과 LLH 계산
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
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()