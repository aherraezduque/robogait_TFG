#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import torch
import numpy as np
import cv2
from ultralytics import YOLO

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped 
from cv_bridge import CvBridge


class HumanDistanceYOLO(Node):
    def __init__(self):
        super().__init__('human_distance_yolo')

        # Parameters
        self.declare_parameter('rgb_topic', '/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/image')
        self.declare_parameter('depth_topic', '/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/depth_image')
        self.declare_parameter('output_topic', '/user_coordinates')
        self.declare_parameter('position_topic', '/user_position')

        self.declare_parameter('model', 'yolo11n-seg.pt')
        #self.declare_parameter('model', 'yolov8n-seg.pt')
        self.declare_parameter('device', 'cuda')  # cuda or cpu

        self.declare_parameter('fx', 504.37)
        self.declare_parameter('fy', 504.22)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.declare_parameter('camera_frame', 'camera_optical_frame')

        self.rgb_topic_ = self.get_parameter('rgb_topic').value
        self.depth_topic_ = self.get_parameter('depth_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.position_topic_ = self.get_parameter('position_topic').value

        self.fx_ = float(self.get_parameter('fx').value)
        self.fy_ = float(self.get_parameter('fy').value)
        self.cx_ = float(self.get_parameter('cx').value)
        self.cy_ = float(self.get_parameter('cy').value)
        self.camera_frame_ = self.get_parameter('camera_frame').value


        self.bridge_ = CvBridge()
        # Image buffers
        self.rgb_image_ = None   
        self.depth_image_ = None

        # Subscribers and publishers
        self.sub_rgb_ = self.create_subscription(Image, self.rgb_topic_, self.rgb_callback, 10)                      
        self.sub_depth_ = self.create_subscription(Image, self.depth_topic_, self.depth_callback, 10)
            
        self.pub_distance_ = self.create_publisher(Float64, self.output_topic_, 10)
        self.pub_position_ = self.create_publisher(PointStamped, self.position_topic_, 10)


        # Load YOLO segmentation model
        model_path = self.get_parameter('model').value
        requested_device = self.get_parameter('device').value

        if requested_device == "cuda" and not torch.cuda.is_available():
            self.get_logger().warn("CUDA requested but not available. Using CPU")
            device = "cpu"
        else:
            device = requested_device
            
        self.model_ = YOLO(model_path)
        self.model_.to(device)

        self.get_logger().info(f"YOLO loaded on {device}, model: {model_path}")



    # Camera topic callbacks
    def rgb_callback(self, msg):
        self.rgb_image_ = self.bridge_.imgmsg_to_cv2(msg, "bgr8")
        self.process_if_ready()

    def depth_callback(self, msg):
        # Depth image in float32 format (m)
        self.depth_image_ = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self.process_if_ready()




    def process_if_ready(self):

        if self.rgb_image_ is None or self.depth_image_ is None:
            return

        # Run YOLO inference
        results = self.model_(self.rgb_image_, verbose=False)

        if len(results[0].boxes) == 0:
            self.get_logger().info("No person detected")
            return

        # Filter detections belonging to the "person" class (COCO class 0)
        persons = [i for i, c in enumerate(results[0].boxes.cls) if int(c) == 0]
        if not persons:
            self.get_logger().info("No person detected")
            return

        # Select the largest bounding box
        areas = []
        for i in persons:
            x1, y1, x2, y2 = results[0].boxes.xyxy[i].cpu().numpy()
            areas.append((i, (x2-x1)*(y2-y1)))
        idx = max(areas, key=lambda x: x[1])[0]

        # Get the segmentation mask of the largest detected person
        mask = results[0].masks.data[idx].cpu().numpy()  # (H, W) values in range  [0,1]
        mask_resized = cv2.resize(mask, (self.rgb_image_.shape[1], self.rgb_image_.shape[0]))
        # 1 = person,  0 = background, pixels with value greater than 0.5 are considered part of the person
        mask_bin = (mask_resized > 0.5).astype(np.uint8)

        depth_clean = np.nan_to_num(
            self.depth_image_,
            nan= 0.0,
            posinf= 0.0,
            neginf= 0.0
        )

        # Apply the segmentation mask to the depth image
        depth_masked = depth_clean * mask_bin

        # Extract valid depth values
        valid_mask = (depth_masked > 0.1) & (depth_masked < 10.0)

        if not np.any(valid_mask):
            self.get_logger().warn("No valid depth values")
            return




        # Pixel coordinates with valid depth values
        v_coords, u_coords = np.where(valid_mask)  # v = row (y), u = column (x)
        Zs = depth_masked[valid_mask]

        # Estimate distance using the median depth 
        distance = float(np.median(Zs))

        # Pinhole camera model: x = ((u - cx) * Z / fx ),   y = ((v - cy) * Z / fy)
        Xs = (u_coords - self.cx_) * Zs / self.fx_
        Ys = (v_coords - self.cy_) * Zs / self.fy_
        
        X = float(np.median(Xs))
        Y = float(np.median(Ys))
        Z = float(np.median(Zs))

        # Publish distance 
        msg = Float64()
        msg.data = distance
        self.pub_distance_.publish(msg)
        self.get_logger().info(f"Distance = {distance:.2f} m")

        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = self.camera_frame_
        pt_msg.point.x = X
        pt_msg.point.y = Y
        pt_msg.point.z = Z
        self.pub_position_.publish(pt_msg)

        self.get_logger().info(
            f"Person detected → Position (X,Y,Z)=({X:.2f},{Y:.2f},{Z:.2f}) m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumanDistanceYOLO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
