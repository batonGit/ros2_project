#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import paho.mqtt.client as mqtt
import json

class GPSMQTTNode(Node):
    def __init__(self):
        super().__init__(node_name='gps_mqtt_node')
        # Публикация координат пункта назначения
        self.dest_publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        # Публикация координат тележки
        self.robot_publisher_ = self.create_publisher(NavSatFix, '/robot/gps/fix', 10)

        # Клиент для пункта назначения
        self.dest_mqtt_client = mqtt.Client()
        self.dest_mqtt_client.on_connect = self.on_dest_connect
        self.dest_mqtt_client.on_message = self.on_dest_message
        self.dest_mqtt_client.connect("test.mosquitto.org", 1883, 60)
        self.dest_mqtt_client.loop_start()

        # Клиент для тележки
        self.robot_mqtt_client = mqtt.Client()
        self.robot_mqtt_client.on_connect = self.on_robot_connect
        self.robot_mqtt_client.on_message = self.on_robot_message
        self.robot_mqtt_client.connect("test.mosquitto.org", 1883, 60)
        self.robot_mqtt_client.loop_start()

    def on_dest_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker for destination")
        client.subscribe("owntracks/user/hncrtm1")

    def on_dest_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        gps_msg = NavSatFix()
        gps_msg.latitude = float(data.get("lat", 0.0))
        gps_msg.longitude = float(data.get("lon", 0.0))
        gps_msg.altitude = float(data.get("alt", 0.0))
        self.dest_publisher_.publish(gps_msg)
        self.get_logger().info(f"Published destination GPS: {gps_msg.latitude}, {gps_msg.longitude}")

    def on_robot_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker for robot")
        client.subscribe("owntracks/user/hwhryh")  # Укажите топик для телефона на тележке

    def on_robot_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        gps_msg = NavSatFix()
        gps_msg.latitude = float(data.get("lat", 0.0))
        gps_msg.longitude = float(data.get("lon", 0.0))
        gps_msg.altitude = float(data.get("alt", 0.0))
        self.robot_publisher_.publish(gps_msg)
        self.get_logger().info(f"Published robot GPS: {gps_msg.latitude}, {gps_msg.longitude}")

    def destroy_node(self):
        self.dest_mqtt_client.loop_stop()
        self.robot_mqtt_client.loop_stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = GPSMQTTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
