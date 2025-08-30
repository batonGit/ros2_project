#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import paho.mqtt.client as mqtt
import json

class GPSMQTTNode(Node):
    def __init__(self):
        super().__init__(node_name='gps_mqtt_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=1)

        # Публикатор для координат пункта назначения (остается без изменений)
        self.dest_publisher_ = self.create_publisher(NavSatFix, '/gps/fix', qos_profile)
        
        # --- ИЗМЕНЕНИЕ: Добавляем обратно публикатор для GPS с телефона тележки, но в новый топик ---
        self.robot_mqtt_publisher_ = self.create_publisher(NavSatFix, '/robot/gps/fix_mqtt', qos_profile)

        # Клиент для пункта назначения
        self.dest_mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.dest_mqtt_client.username_pw_set("my_mqtt", "Hnw8dbbhw") # Убедитесь, что данные верны
        self.dest_mqtt_client.on_connect = self.on_dest_connect
        self.dest_mqtt_client.on_message = self.on_dest_message
        self.dest_mqtt_client.on_disconnect = lambda client, userdata, rc: self.get_logger().warn(f"Destination MQTT client disconnected with code: {rc}")
        try:
            self.dest_mqtt_client.connect("megalitour.ru", 1883, 60)
            self.dest_mqtt_client.loop_start()
            self.get_logger().info("Destination MQTT client loop started.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect destination MQTT client: {e}")

        # --- ИЗМЕНЕНИЕ: Возвращаем MQTT клиент для GPS тележки ---
        self.robot_mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.robot_mqtt_client.username_pw_set("gps_robot", "Hnw8dbbhw") # Убедитесь, что данные верны
        self.robot_mqtt_client.on_connect = self.on_robot_connect
        self.robot_mqtt_client.on_message = self.on_robot_message
        self.robot_mqtt_client.on_disconnect = lambda client, userdata, rc: self.get_logger().warn(f"Robot MQTT client disconnected with code: {rc}")
        try:
            self.robot_mqtt_client.connect("megalitour.ru", 1883, 60)
            self.robot_mqtt_client.loop_start()
            self.get_logger().info("Robot MQTT client loop started.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect robot MQTT client: {e}")
        # --- КОНЕЦ ИЗМЕНЕНИЯ ---

    # ... (методы on_dest_connect, process_mqtt_message, on_dest_message остаются без изменений) ...
    def on_dest_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Successfully connected to MQTT broker for destination.")
            client.subscribe("owntracks/my_mqtt/hncrtm1") 
            self.get_logger().info("Subscribed to owntracks/my_mqtt/hncrtm1 for destination.")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker for destination, return code {rc}")
    
    def process_mqtt_message(self, payload_str, topic_name, frame_id_prefix="gps_link"):
        try: data = json.loads(payload_str)
        except json.JSONDecodeError as e: self.get_logger().error(f"JSON Decode Error: {e}"); return None
        if data.get("_type") != "location": return None
        try: lat, lon = float(data["lat"]), float(data["lon"])
        except (KeyError, ValueError, TypeError): self.get_logger().error("Invalid lat/lon"); return None
        if abs(lat) < 0.0001 and abs(lon) < 0.0001: self.get_logger().warn("Zero coords received"); return None
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = frame_id_prefix
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        accuracy_m = float(data.get("acc", 9999.0))
        if 0 < accuracy_m < 50.0: gps_msg.status.status = NavSatStatus.STATUS_FIX
        else: gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
        gps_msg.latitude, gps_msg.longitude = lat, lon
        gps_msg.altitude = float(data.get("alt", 0.0))
        if gps_msg.status.status == NavSatStatus.STATUS_FIX:
            h_var = accuracy_m**2; v_var = (accuracy_m * 2)**2
            gps_msg.position_covariance = [h_var, 0.0, 0.0, 0.0, h_var, 0.0, 0.0, 0.0, v_var]
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return gps_msg

    def on_dest_message(self, client, userdata, msg):
        gps_msg = self.process_mqtt_message(msg.payload.decode(), msg.topic, "destination_gps_antenna")
        if gps_msg:
            self.dest_publisher_.publish(gps_msg)
            self.get_logger().info(f"Published to /gps/fix (Dest): Lat {gps_msg.latitude:.5f}, Lon {gps_msg.longitude:.5f}")

    # --- ИЗМЕНЕНИЕ: Возвращаем логику для GPS тележки ---
    def on_robot_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Successfully connected to MQTT broker for robot.")
            client.subscribe("owntracks/gps_robot/hwhryh") # Убедитесь, что топик верный
            self.get_logger().info("Subscribed to owntracks/gps_robot/hwhryh for robot.")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker for robot, return code {rc}")

    def on_robot_message(self, client, userdata, msg):
        gps_msg = self.process_mqtt_message(msg.payload.decode(), msg.topic, "robot_gps_mqtt")
        if gps_msg:
            self.robot_mqtt_publisher_.publish(gps_msg) # Публикуем в /robot/gps/fix_mqtt
            self.get_logger().info(f"Published to /robot/gps/fix_mqtt (Robot): Lat {gps_msg.latitude:.5f}, Lon {gps_msg.longitude:.5f}")
    # --- КОНЕЦ ИЗМЕНЕНИЯ ---

    def destroy_node(self):
        self.get_logger().info("Stopping MQTT client loops and shutting down GPSMQTTNode.")
        if self.dest_mqtt_client:
            self.dest_mqtt_client.loop_stop()
            self.dest_mqtt_client.disconnect()
        # --- ИЗМЕНЕНИЕ: Добавляем остановку для клиента робота ---
        if self.robot_mqtt_client:
            self.robot_mqtt_client.loop_stop()
            self.robot_mqtt_client.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = GPSMQTTNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("GPSMQTTNode stopped.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()