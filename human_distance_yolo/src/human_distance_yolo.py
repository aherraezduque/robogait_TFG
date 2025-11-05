#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import torch
import numpy as np
import cv2
from ultralytics import YOLO

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge


class HumanDistanceYOLO(Node):
    def __init__(self):
        super().__init__('human_distance_yolo')

        # Parmetros
        self.declare_parameter('rgb_topic', '/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/image')
        self.declare_parameter('depth_topic', '/world/mi_mundo/model/rover_mini/link/base_link/sensor/rgbd_camera/depth_image')
        self.declare_parameter('output_topic', '/user_coordinates')
        self.declare_parameter('model', 'yolo11n-seg.pt')
        #self.declare_parameter('model', 'yolov8n-seg.pt')
        self.declare_parameter('device', 'cuda')  # o cpu

        self.rgb_topic_ = self.get_parameter('rgb_topic').value
        self.depth_topic_ = self.get_parameter('depth_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value

        self.bridge = CvBridge()
        # Buffers para las imagenes
        self.rgb_image = None   
        self.depth_image = None

        # Subs y pubs
        self.sub_rgb = self.create_subscription(Image, self.rgb_topic_, self.rgb_callback, 10)
                        
        self.sub_depth = self.create_subscription(Image, self.depth_topic_, self.depth_callback, 10)
            
        self.pub_distance = self.create_publisher(Float64, self.output_topic_, 10)

        # Cargar modelo YOLOv8 
        model_path = self.get_parameter('model').value
        device = self.get_parameter('device').value
        self.model = YOLO(model_path)
        self.model.to(device)

        self.get_logger().info(f"YOLOv8 cargado en {device}, modelo: {model_path}")



    # Callbacks de los topics de la camara
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_if_ready()

    def depth_callback(self, msg):
        # Deoth image en formato float32 (m)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self.process_if_ready()




    def process_if_ready(self):

        if self.rgb_image is None or self.depth_image is None:
            return

        # Deteccion con YOLOv8 ---
        results = self.model(self.rgb_image, verbose=False)

        if len(results[0].boxes) == 0:
            self.get_logger().info("No se detecto persona")
            return

        # Filtrar detecciones de clase "person" (clase 'c' 0 en COCO)
        persons = [i for i, c in enumerate(results[0].boxes.cls) if int(c) == 0]
        if not persons:
            self.get_logger().info("No se detecto persona")
            return

        # Coger el bounding box mas grande 
        areas = []
        for i in persons:
            x1, y1, x2, y2 = results[0].boxes.xyxy[i].cpu().numpy()
            areas.append((i, (x2-x1)*(y2-y1)))
        idx = max(areas, key=lambda x: x[1])[0]

        # Obtener mascara de la persona mas grande
        mask = results[0].masks.data[idx].cpu().numpy()  # (H, W) en rango [0,1]
        mask_resized = cv2.resize(mask, (self.rgb_image.shape[1], self.rgb_image.shape[0]))
            # 1 (persona) 0 (fondo), si pasa de 0.5 se da por perteneciente a la persona (delimitar la persona)
        mask_bin = (mask_resized > 0.5).astype(np.uint8)

        # Aplicar mascara a imagen de profundidad 
        depth_masked = self.depth_image * mask_bin

        # Extraer valores validos
        valid_depths = depth_masked[(depth_masked > 0.1) & (depth_masked < 10.0)]  # rango [0.1m, 10m]
        if len(valid_depths) == 0:
            self.get_logger().warn("Depth values NO validos")
            return

        # Calcular distancia con la mediana
        distance = float(np.median(valid_depths))

        # Publicar en el topic
        msg = Float64()
        msg.data = distance
        self.pub_distance.publish(msg)

        self.get_logger().info(f"Distancia =: {distance:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = HumanDistanceYOLO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
