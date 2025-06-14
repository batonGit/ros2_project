#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
import sys # Для завершения работы при ошибке

# Импортируем наши новые кастомные сообщения
from vision_processing.msg import Detection, Detections

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # --- Параметры ---
        self.declare_parameter('model_path', 'yolov8n.pt') # Путь к файлу весов модели YOLO
        self.declare_parameter('input_image_topic', 'kinect_image') # Топик с изображениями от Kinect
        self.declare_parameter('detection_image_topic', '/yolo/detection_image') # Топик для отладочного изображения
        self.declare_parameter('detections_topic', '/yolo/detections') # Топик для структурированных данных
        self.declare_parameter('confidence_threshold', 0.5) # Порог уверенности для детекции

        # Получаем значения параметров
        model_path = self.get_parameter('model_path').value
        input_image_topic = self.get_parameter('input_image_topic').value
        detection_image_topic = self.get_parameter('detection_image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        
        # Инициализация YOLO
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO model '{model_path}' loaded successfully.")
            self.class_names = self.model.names
            self.get_logger().info(f"Model classes available: {list(self.class_names.values())}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model from '{model_path}': {e}")
            # Завершаем работу, если модель не загрузилась, т.к. узел бесполезен
            # sys.exit() может быть слишком резким, лучше дать main() завершиться
            self.model = None # Устанавливаем в None, чтобы callback'и не пытались его использовать
            return

        self.bridge = CvBridge()
        
        # Подписка на изображения от Kinect
        self.subscription = self.create_subscription(
            Image,
            input_image_topic,
            self.image_callback,
            10 # QoS по умолчанию
        )
        self.get_logger().info(f"Subscribed to image topic: '{input_image_topic}'")
        
        # Публикация аннотированного изображения для отладки
        self.detection_image_publisher = self.create_publisher(Image, detection_image_topic, 10)
        
        # Публикация структурированных данных о всех детекциях
        self.detections_publisher = self.create_publisher(Detections, detections_topic, 10)
        
        self.get_logger().info(f"YOLO Node initialized. Publishing detections to '{detections_topic}'. Confidence threshold: {self.conf_threshold}")

    def image_callback(self, msg: Image):
        # Если модель не загрузилась при инициализации, ничего не делаем
        if not hasattr(self, 'model') or self.model is None:
            return

        self.get_logger().debug('Received an image frame.')
        try:
            # Конвертация ROS Image в OpenCV формат BGR8
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error converting image: {e}')
            return
            
        # Запуск детекции YOLO
        # verbose=False чтобы не было лишнего вывода YOLO в консоль
        results = self.model(cv_image, verbose=False)
        
        # --- Обработка результатов и публикация структурированных данных ---
        
        # Создаем сообщение-массив для всех детекций в этом кадре
        detections_msg = Detections()
        detections_msg.header = msg.header # Используем тот же заголовок, что и у входного изображения
        
        # Итерируемся по всем обнаруженным объектам в кадре
        for box in results[0].boxes:
            confidence = float(box.conf)
            
            # Отбрасываем детекции с низкой уверенностью
            if confidence < self.conf_threshold:
                continue

            class_id = int(box.cls)
            class_name = self.class_names.get(class_id, 'unknown')
            
            # Получаем нормализованные координаты рамки (x_center, y_center, width, height)
            # Они в диапазоне от 0.0 до 1.0, что удобно для логики управления
            try:
                normalized_box = box.xywhn[0]
            except IndexError:
                continue # Пропускаем, если рамка пустая

            # Создаем сообщение для одной детекции
            detection_data = Detection()
            detection_data.class_name = class_name
            detection_data.class_id = class_id
            detection_data.confidence = confidence
            detection_data.x_center_norm = float(normalized_box[0])
            detection_data.y_center_norm = float(normalized_box[1])
            detection_data.width_norm = float(normalized_box[2])
            detection_data.height_norm = float(normalized_box[3])
            
            # Добавляем эту детекцию в массив
            detections_msg.detections.append(detection_data)

        # Публикуем сообщение с массивом всех найденных объектов
        self.detections_publisher.publish(detections_msg)
        if len(detections_msg.detections) > 0:
            self.get_logger().info(f"Detected {len(detections_msg.detections)} objects with confidence > {self.conf_threshold}")

        
        # --- Публикация отладочного изображения с рамками ---
        annotated_image = results[0].plot()
        
        try:
            # Конвертируем обратно в ROS Image и публикуем
            detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            detection_img_msg.header = msg.header
            self.detection_image_publisher.publish(detection_img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error publishing annotated image: {e}')


    def destroy_node(self):
        self.get_logger().info("YOLO Node shutting down.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    
    # Проверяем, был ли узел успешно инициализирован (загрузилась ли модель)
    if hasattr(yolo_node, 'model') and yolo_node.model is not None:
        try:
            rclpy.spin(yolo_node)
        except KeyboardInterrupt:
            yolo_node.get_logger().info("YOLO node stopped by user.")
    else:
        # Лог об ошибке загрузки модели уже был в __init__
        yolo_node.get_logger().fatal("Node initialization failed (likely model not found). Shutting down.")

    # Корректное завершение работы
    if rclpy.ok():
        if 'yolo_node' in locals() and yolo_node.executor: # Проверка, что узел все еще валиден
             yolo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()