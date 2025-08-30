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

        # Публикуем ТОЛЬКО координаты пункта назначения
        self.dest_publisher_ = self.create_publisher(NavSatFix, '/gps/fix', qos_profile)

        # Клиент ТОЛЬКО для пункта назначения
        self.dest_mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        # Укажите ваши реальные логин/пароль, если они нужны
        self.dest_mqtt_client.username_pw_set("my_mqtt", "Hnw8dbbhw") 
        self.dest_mqtt_client.on_connect = self.on_dest_connect
        self.dest_mqtt_client.on_message = self.on_dest_message
        self.dest_mqtt_client.on_disconnect = lambda client, userdata, rc: self.get_logger().warn(f"Destination MQTT client disconnected with result code: {rc}")
        try:
            self.dest_mqtt_client.connect("megalitour.ru", 1883, 60) # Укажите ваш брокер
            self.dest_mqtt_client.loop_start()
            self.get_logger().info("Destination-only MQTT client loop started.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect destination MQTT client: {e}")

    def on_dest_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Successfully connected to MQTT broker for destination.")
            client.subscribe("owntracks/my_mqtt/hncrtm1") # Укажите ваш топик назначения
            self.get_logger().info("Subscribed to destination topic.")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker for destination, return code {rc}")

    # Метод process_mqtt_message остается таким же, как был
    def process_mqtt_message(self, payload_str, topic_name, frame_id_prefix="gps_link"):
        # ... (скопируйте ваш существующий метод process_mqtt_message сюда) ...
        # ... (я не буду его здесь повторять для краткости, он у вас уже есть) ...
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

    def destroy_node(self):
        self.get_logger().info("Stopping MQTT client loop.")
        if self.dest_mqtt_client:
            self.dest_mqtt_client.loop_stop()
            self.dest_mqtt_client.disconnect()
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