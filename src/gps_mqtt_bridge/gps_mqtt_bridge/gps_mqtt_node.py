#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus # Добавлен NavSatStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # Для QoS
import paho.mqtt.client as mqtt
import json

class GPSMQTTNode(Node):
    def __init__(self):
        super().__init__(node_name='gps_mqtt_node')

        # Определение QoS профиля
        # RELIABLE: Гарантирует доставку, если возможно.
        # KEEP_LAST: Хранит только указанное количество последних сообщений (depth).
        # VOLATILE: Сообщения не сохраняются для "опоздавших" подписчиков.
        # Для GPS данных, которые часто обновляются, VOLATILE обычно подходит.
        # Если нужно, чтобы новый подписчик сразу получил последнее значение, можно использовать TRANSIENT_LOCAL.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, 
            depth=1 
        )

        # Публикация координат пункта назначения
        self.dest_publisher_ = self.create_publisher(NavSatFix, '/gps/fix', qos_profile)
        # Публикация координат тележки
        self.robot_publisher_ = self.create_publisher(NavSatFix, '/robot/gps/fix', qos_profile)

        # Клиент для пункта назначения
        self.dest_mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1) # Явно указываем версию API
        self.dest_mqtt_client.username_pw_set("my_mqtt", "Hnw8dbbhw")
        self.dest_mqtt_client.on_connect = self.on_dest_connect
        self.dest_mqtt_client.on_message = self.on_dest_message
        # Можно добавить on_disconnect для отладки
        self.dest_mqtt_client.on_disconnect = lambda client, userdata, rc: self.get_logger().warn(f"Destination MQTT client disconnected with result code: {rc}")
        try:
            self.dest_mqtt_client.connect("megalitour.ru", 1883, 60)
            self.dest_mqtt_client.loop_start()
            self.get_logger().info("Destination MQTT client loop started.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect destination MQTT client: {e}")


        # Клиент для тележки
        self.robot_mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1) # Явно указываем версию API
        self.robot_mqtt_client.username_pw_set("gps_robot", "Hnw8dbbhw")
        self.robot_mqtt_client.on_connect = self.on_robot_connect
        self.robot_mqtt_client.on_message = self.on_robot_message
        self.robot_mqtt_client.on_disconnect = lambda client, userdata, rc: self.get_logger().warn(f"Robot MQTT client disconnected with result code: {rc}")
        try:
            self.robot_mqtt_client.connect("megalitour.ru", 1883, 60)
            self.robot_mqtt_client.loop_start()
            self.get_logger().info("Robot MQTT client loop started.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect robot MQTT client: {e}")


    def on_dest_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Successfully connected to MQTT broker for destination.")
            # Топик для телефона пункта назначения
            client.subscribe("owntracks/my_mqtt/hncrtm1") 
            self.get_logger().info("Subscribed to owntracks/my_mqtt/hncrtm1 for destination.")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker for destination, return code {rc}")

    def process_mqtt_message(self, payload_str, topic_name, frame_id_prefix="gps_link"):
        try:
            data = json.loads(payload_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON from MQTT topic {topic_name}: {e}. Payload: '{payload_str[:100]}'")
            return None # Возвращаем None, если не можем распарсить

        # Проверяем, что это сообщение о местоположении от OwnTracks
        if data.get("_type") != "location":
            self.get_logger().debug(f"Received non-location message on {topic_name}: Type '{data.get('_type')}'. Skipping.")
            return None

        try:
            lat = float(data.get("lat", 0.0))
            lon = float(data.get("lon", 0.0))
        except (ValueError, TypeError) as e:
            self.get_logger().error(f"Invalid lat/lon format in message from {topic_name}: {e}. Data: {data}")
            return None

        # Фильтруем невалидные или нулевые координаты (0,0 часто означает ошибку)
        # Порог можно настроить; для большинства мест на Земле широта/долгота не будут одновременно так близки к 0.
        if abs(lat) < 0.0001 and abs(lon) < 0.0001:
            self.get_logger().warn(f"Received (0,0) or near-zero coordinates on {topic_name}, likely invalid. Skipping. Data: {data}")
            return None
            
        gps_msg = NavSatFix()
        
        # 1. Заполняем Header
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = frame_id_prefix # Пример: 'destination_gps_link' или 'robot_gps_link'

        # 2. Заполняем Status
        gps_msg.status.service = NavSatStatus.SERVICE_GPS # Указываем, что это GPS

        # Определяем статус фиксации на основе точности 'acc' (если есть)
        # 'acc' в OwnTracks - это горизонтальная точность в метрах.
        accuracy_m = float(data.get("acc", 9999.0)) # Большое значение по умолчанию, если нет 'acc'

        if 0 < accuracy_m < 50.0: # Считаем хорошим фиксом, если точность < 50 метров
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
            self.get_logger().warn(f"No fix or poor accuracy ({accuracy_m}m) from {topic_name}.")

        # 3. Координаты
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        try:
            gps_msg.altitude = float(data.get("alt", 0.0))
        except (ValueError, TypeError):
            gps_msg.altitude = 0.0
            self.get_logger().warn(f"Invalid altitude format from {topic_name}, using 0.0. Data: {data}")


        # 4. Ковариация (упрощенно)
        if gps_msg.status.status == NavSatStatus.STATUS_FIX:
            # Примерное заполнение диагональной ковариации на основе 'acc'
            # variance = accuracy^2. Ковариация в метрах^2 (ENU: East, North, Up)
            horizontal_variance = accuracy_m**2
            vertical_variance = (accuracy_m * 2)**2 # Вертикальная точность часто хуже

            gps_msg.position_covariance[0] = horizontal_variance  # East^2
            gps_msg.position_covariance[4] = horizontal_variance  # North^2
            gps_msg.position_covariance[8] = vertical_variance    # Up^2
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            # Если нет фикса или точность неизвестна/плохая
            large_variance = 1e9 # Большое значение для индикации высокой неопределенности
            gps_msg.position_covariance[0] = large_variance
            gps_msg.position_covariance[4] = large_variance
            gps_msg.position_covariance[8] = large_variance
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
        return gps_msg


    def on_dest_message(self, client, userdata, msg):
        self.get_logger().debug(f"Raw Dest MQTT ({msg.topic}): {msg.payload.decode()[:100]}") # Лог первых 100 символов
        gps_msg = self.process_mqtt_message(msg.payload.decode(), msg.topic, "destination_gps_antenna")
        if gps_msg:
            self.dest_publisher_.publish(gps_msg)
            self.get_logger().info(f"Published to /gps/fix (Dest): Lat {gps_msg.latitude:.5f}, Lon {gps_msg.longitude:.5f}, Alt {gps_msg.altitude:.1f}, Status {gps_msg.status.status}")

    def on_robot_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Successfully connected to MQTT broker for robot.")
            # Топик для телефона на тележке
            client.subscribe("owntracks/gps_robot/hwhryh") 
            self.get_logger().info("Subscribed to owntracks/user/hwhryh for robot.")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker for robot, return code {rc}")

    def on_robot_message(self, client, userdata, msg):
        self.get_logger().debug(f"Raw Robot MQTT ({msg.topic}): {msg.payload.decode()[:100]}")
        gps_msg = self.process_mqtt_message(msg.payload.decode(), msg.topic, "robot_gps_antenna")
        if gps_msg:
            self.robot_publisher_.publish(gps_msg)
            self.get_logger().info(f"Published to /robot/gps/fix (Robot): Lat {gps_msg.latitude:.5f}, Lon {gps_msg.longitude:.5f}, Alt {gps_msg.altitude:.1f}, Status {gps_msg.status.status}")

    def destroy_node(self):
        self.get_logger().info("Stopping MQTT client loops and shutting down GPSMQTTNode.")
        if self.dest_mqtt_client:
            self.dest_mqtt_client.loop_stop()
            self.dest_mqtt_client.disconnect()
        if self.robot_mqtt_client:
            self.robot_mqtt_client.loop_stop()
            self.robot_mqtt_client.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = GPSMQTTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("GPSMQTTNode stopped by user.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()