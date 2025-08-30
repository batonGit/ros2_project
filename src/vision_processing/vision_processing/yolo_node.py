#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
import sys

# --- НОВОЕ: Импорты для синхронизации сообщений ---
import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Импортируем наши обновленные кастомные сообщения
from vision_interfaces.msg import Detection, Detections

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # --- Параметры ---
        self.declare_parameter('model_path', 'yolov8n.pt')
        # Имена топиков по умолчанию соответствуют выводу стандартного драйвера kinect_ros2
        self.declare_parameter('rgb_image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('detection_image_topic', '/yolo/detection_image')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('confidence_threshold', 0.6) # Немного поднимем порог для надежности
        self.declare_parameter('target_class_name', 'traffic cone')

        # Получаем значения параметров
        model_path = self.get_parameter('model_path').value
        rgb_topic = self.get_parameter('rgb_image_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        detection_image_topic = self.get_parameter('detection_image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_class_name = self.get_parameter('target_class_name').value
        
        # Инициализация YOLO
        try:
            self.model = YOLO(model_path)
            self.class_names = self.model.names
            self.get_logger().info(f"YOLO model '{model_path}' loaded successfully.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model: {e}")
            self.model = None
            return

        self.bridge = CvBridge()
        
        # --- ИЗМЕНЕНИЕ: Настройка синхронизированных подписчиков ---
        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        
        # TimeSynchronizer будет вызывать self.sync_callback, когда получит сообщения
        # с одинаковой временной меткой из обоих топиков.
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info(f"Subscribed to RGB topic: '{rgb_topic}'")
        self.get_logger().info(f"Subscribed to Depth topic: '{depth_topic}'")
        # --- КОНЕЦ ИЗМЕНЕНИЯ ---
        
        self.detection_image_publisher = self.create_publisher(Image, detection_image_topic, 10)
        self.detections_publisher = self.create_publisher(Detections, detections_topic, 10)
        
        self.get_logger().info(f"YOLO Node initialized. Target class: '{self.target_class_name}', Confidence: {self.conf_threshold}")

    # --- ИЗМЕНЕНИЕ: Новый синхронизированный callback ---
    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if not hasattr(self, 'model') or self.model is None:
            return

        self.get_logger().debug('Received a synchronized pair of RGB and Depth images.')
        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # Данные глубины от Kinect - это 16-битные целые числа без знака (в мм)
            cv_image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
            
        results = self.model(cv_image_rgb, verbose=False)
        
        detections_msg = Detections()
        detections_msg.header = rgb_msg.header
        
        # Получаем размеры изображения для денормализации координат
        img_height, img_width, _ = cv_image_rgb.shape
        
        for box in results[0].boxes:
            confidence = float(box.conf)
            if confidence < self.conf_threshold:
                continue

            class_id = int(box.cls)
            class_name = self.class_names.get(class_id, 'unknown')
            
            # --- НОВОЕ: Вычисление расстояния до объекта ---
            distance_m = 0.0
            
            # Получаем нормализованные координаты рамки [x_center, y_center, width, height]
            normalized_box = box.xywhn[0]
            # Преобразуем в пиксельные координаты центра рамки
            cx_pixel = int(normalized_box[0] * img_width)
            cy_pixel = int(normalized_box[1] * img_height)

            # Проверяем, что координаты находятся в пределах изображения
            if 0 <= cy_pixel < img_height and 0 <= cx_pixel < img_width:
                # Считываем значение глубины (в мм) из изображения глубины
                depth_mm = cv_image_depth[cy_pixel, cx_pixel]
                # Kinect возвращает 0, если расстояние не может быть определено
                if depth_mm > 0:
                    distance_m = float(depth_mm) / 1000.0 # Конвертируем в метры
            # --- КОНЕЦ ВЫЧИСЛЕНИЯ РАССТОЯНИЯ ---

            detection_data = Detection()
            detection_data.class_name = class_name
            detection_data.class_id = class_id
            detection_data.confidence = confidence
            detection_data.x_center_norm = float(normalized_box[0])
            detection_data.y_center_norm = float(normalized_box[1])
            detection_data.width_norm = float(normalized_box[2])
            detection_data.height_norm = float(normalized_box[3])
            detection_data.distance_m = distance_m # Заполняем новое поле
            
            detections_msg.detections.append(detection_data)

        self.detections_publisher.publish(detections_msg)
        if len(detections_msg.detections) > 0:
            self.get_logger().info(f"Detected {len(detections_msg.detections)} objects. Target distance: {detections_msg.detections[0].distance_m:.2f}m (if found)")

        
        # Публикация отладочного изображения
        annotated_image = results[0].plot()
        try:
            detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            detection_img_msg.header = rgb_msg.header
            self.detection_image_publisher.publish(detection_img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error publishing annotated image: {e}')

    def destroy_node(self):
        self.get_logger().info("YOLO Node shutting down.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    if hasattr(yolo_node, 'model') and yolo_node.model is not None:
        try:
            rclpy.spin(yolo_node)
        except KeyboardInterrupt:
            yolo_node.get_logger().info("YOLO node stopped by user.")
        finally:
            if rclpy.ok():
                if yolo_node.is_valid(): yolo_node.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
    main()