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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from scipy.optimize import minimize
from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.linalg import eigvals

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_data_node')
        self.pub = self.create_publisher(PoseStamped, 'tag', 10)
        
        self.declare_parameter('system_id_list', [0])
        self.system_id_list = self.get_parameter('system_id_list').get_parameter_value().integer_array_value
        
        self.topic_anchor_prefix = "/uwb/anchor_"
        self.topic_imu_prefix = "/iris/id_"
        
        self.dictected_anchor_pos = np.zeros((len(self.system_id_list), 3))
        self.dictedted_anchor_ranging = np.zeros(len(self.system_id_list))
        
        for sys_id in self.system_id_list:
            globals()["self.anchor_{}_subscriber".format(sys_id)] = self.create_subscription(Ranging, f'{self.topic_anchor_prefix}{sys_id}/ranging', self.uwd_data_callback, 10)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def uwd_data_callback(self, msg):
        anchorID = msg.anchor_id
        anchor_pose = np.array([msg.anchor_pose.position.x, msg.anchor_pose.position.y, msg.anchor_pose.position.z])
        self.dictected_anchor_pos[anchorID - 1] = anchor_pose
        self.dictedted_anchor_ranging[anchorID - 1] = msg.range / 1000
        
        robot_pos=[]
        if len(self.dictected_anchor_pos)!=0:
            robot_pos = self.position_calculation(self.dictected_anchor_pos, self.dictedted_anchor_ranging)
        
        if robot_pos is not None and len(robot_pos) > 0:
            self.publish_data(robot_pos[0], robot_pos[1],robot_pos[2])

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

        # self.get_logger().info(f"Publishing: {robot_pos.pose.position.x, robot_pos.pose.position.y, robot_pos.pose.position.z}")
        self.pub.publish(robot_pos)
        
def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()