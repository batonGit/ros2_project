#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # <-- ИЗМЕНЕНИЕ: Используем стандартное сообщение String
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import sys
import json # <-- ДОБАВЛЕНО: для работы с JSON

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # --- Параметры ---
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('input_image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('detection_image_topic', '/yolo/detection_image')
        self.declare_parameter('detections_json_topic', '/yolo/detections_json') # <-- ИЗМЕНЕНИЕ: Новый топик
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('target_class_name', 'traffic cone')

        # Получаем значения параметров
        model_path = self.get_parameter('model_path').value
        input_image_topic = self.get_parameter('input_image_topic').value
        detection_image_topic = self.get_parameter('detection_image_topic').value
        detections_json_topic = self.get_parameter('detections_json_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_class_name = self.get_parameter('target_class_name').value
        
        # Инициализация YOLO
        try:
            self.model = YOLO(model_path)
            self.class_names = self.model.names
            self.get_logger().info(f"YOLO model '{model_path}' loaded successfully.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model: {e}")
            self.model = None; return

        self.bridge = CvBridge()
        
        # Подписчик на изображения (без изменений)
        self.subscription = self.create_subscription(Image, input_image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: '{input_image_topic}'")
        
        # Издатель для отладочного изображения (без изменений)
        self.detection_image_publisher = self.create_publisher(Image, detection_image_topic, 10)
        
        # --- ИЗМЕНЕНИЕ: Издатель для JSON-строки ---
        self.detections_publisher = self.create_publisher(String, detections_json_topic, 10)
        
        self.get_logger().info(f"YOLO Node initialized. Publishing detections to '{detections_json_topic}'.")

    def image_callback(self, msg: Image):
        if not hasattr(self, 'model') or self.model is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}'); return
            
        results = self.model(cv_image, verbose=False)
        
        # --- ИЗМЕНЕНИЕ: Формируем список словарей, а не кастомных сообщений ---
        detections_list = []
        
        for box in results[0].boxes:
            confidence = float(box.conf)
            if confidence < self.conf_threshold: continue

            class_id = int(box.cls)
            class_name = self.class_names.get(class_id, 'unknown')
            normalized_box = box.xywhn[0]

            detection_dict = {
                "class_name": class_name,
                "confidence": confidence,
                "x_center_norm": float(normalized_box[0]),
                "y_center_norm": float(normalized_box[1]),
                "width_norm": float(normalized_box[2]),
                "height_norm": float(normalized_box[3]),
            }
            detections_list.append(detection_dict)

        # Публикуем, только если что-то нашли, чтобы не спамить
        if len(detections_list) > 0:
            json_string = json.dumps(detections_list)
            string_msg = String()
            string_msg.data = json_string
            self.detections_publisher.publish(string_msg)
            self.get_logger().info(f"Detected {len(detections_list)} objects. JSON: {json_string}")
        
        # Публикация отладочного изображения (без изменений)
        annotated_image = results[0].plot()
        try:
            detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            detection_img_msg.header = msg.header
            self.detection_image_publisher.publish(detection_img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error publishing annotated image: {e}')

    # ... (destroy_node и main остаются без изменений) ...

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    if hasattr(yolo_node, 'model') and yolo_node.model is not None:
        try: rclpy.spin(yolo_node)
        except KeyboardInterrupt: pass
    else: yolo_node.get_logger().fatal("Node initialization failed.")
    if rclpy.ok():
        yolo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()