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
from gtec_msgs.msg import Ranging, RangingList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from scipy.optimize import minimize
from tf2_ros import TransformBroadcaster

import numpy as np
from scipy.linalg import eigvals

all_distance = []
pose_x = 0
pose_y = 0

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_data_node')
        self.pub = self.create_publisher(PoseStamped, 'tag', 10)
        self.uwb_data_sub = self.create_subscription(
            RangingList,
            '/gtec/toa/id_1/ranging',
            self.uwd_data_callback,
            10
        )
        self.anchor_pos_sub = self.create_subscription(
            MarkerArray,
            '/gtec/toa/id_1/anchors',
            self.anchor_pos_callback,
            10
        )
        self.uwb_data_list = RangingList()
        self.placed_anchor_list = []
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def uwd_data_callback(self, msg):
        all_distance=[]
        ditected_anchor_pos_list=[]
        for anchor in msg.anchors:
            all_distance.append(anchor.range/1000)
            for placed_anchor in self.placed_anchor_list:
                if placed_anchor.anchor_id == anchor.anchor_id:
                    ditected_anchor_pos_list.append([placed_anchor.x, placed_anchor.y, placed_anchor.z])
                    break
        ditected_anchor_pos_list=np.array(ditected_anchor_pos_list) #Transport list to np.array
        all_distance=np.array(all_distance)
        
        robot_pos=[]
        if len(ditected_anchor_pos_list)!=0:
            #robot_pos = self.trilateration(ditected_anchor_pos_list, all_distance)
            robot_pos = self.position_calculation(ditected_anchor_pos_list, all_distance)
        
        if robot_pos is not None and len(robot_pos) > 0:
            self.publish_data(robot_pos[0], robot_pos[1],robot_pos[2])

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
        
    def anchor_pos_callback(self, msg):
        self.placed_anchor_list=[]
        for data in msg.markers:
            anchor_data = Anchor(data.pose.position.x, data.pose.position.y, data.pose.position.z, data.id)
            self.placed_anchor_list.append(anchor_data)
        if len(self.placed_anchor_list) == 0:
            self.get_logger().warn("No anchors found. Retrying...")
            return self.anchor_pos_callback()

class Anchor():
    def __init__(self, x, y, z, anchor_id):
        self.x = x
        self.y = y
        self.z = z
        self.anchor_id = anchor_id
    
    # Getter
    def x(self):
        return self.x
    def y(self):
        return self.y
    def z(self):
        return self.z
    def anchor_id(self):
        return self.anchor_id
    
    #Setter
    def x(self, x):
        self.x = x
    def y(self, y):
        self.y = y
    def z(self, z):
        self.z = z
    def anchor_id(self, anchor_id):
        self.achor_id = anchor_id
        
def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()