#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

from px4_msgs.srv import ModeChange, TrajectorySetpoint as TrajectorySetpointSrv, VehicleCommand as VehicleCommandSrv, GlobalPath

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.declare_parameter('system_id', 1)

        self.model = YOLO('/home/user/Projects/uwb_reconn/src/yolobot_recognition/scripts/yolov8n.pt')
        #self.model = YOLO('/home/suvlab/ros2_ws/forest_recon/src/yolobot_recognition/scripts/yolov8n.pt')
        
        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            #f"/recon_{self.get_parameter('system_id').get_parameter_value().integer_value}/camera/image_raw",
            f"/recon_1/camera/image_raw",
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, f"/Yolov8_Inference_{self.get_parameter('system_id').get_parameter_value().integer_value}", 1)
        self.img_pub = self.create_publisher(Image, f"/inference_result_{self.get_parameter('system_id').get_parameter_value().integer_value}", 1)

        #drone_manager_srv_client
        self.topic_prefix_manager_ = f"drone{self.get_parameter('system_id').get_parameter_value().integer_value}/manager/"
        self.mode_change_client = self.create_client(ModeChange, f'{self.topic_prefix_manager_}srv/mode_change')
    
    def send_request(self, mode):
        request = ModeChange.Request()
        request.suv_mode = mode

        future = self.mode_change_client.call_async(request)
        return future
    
    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)
                if self.inference_result.class_name == "person" and self.inference_result.left >= 50:
                    self.send_request(2)
                    
            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
