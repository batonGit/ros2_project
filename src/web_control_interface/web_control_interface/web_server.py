import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from flask import Flask, render_template, request, jsonify, send_from_directory
import threading
import os

app = Flask(__name__, static_url_path='', static_folder='static')
control_node = None

class ControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Добавляем публикатор для /destination_id
        self.destination_pub = self.create_publisher(Int32, '/destination_id', 10)
        self.get_logger().info('Web Control Node initialized')
        
    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear_x={linear_x}, angular_z={angular_z}')

    def publish_destination(self, destination_id):
        msg = Int32()
        msg.data = int(destination_id)
        self.destination_pub.publish(msg)
        self.get_logger().info(f'Published destination_id: {msg.data}')

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/control', methods=['POST'])
def control():
    data = request.get_json()
    linear_x = data.get('linear_x', 0.0)
    angular_z = data.get('angular_z', 0.0)
    
    if control_node:
        control_node.publish_velocity(linear_x, angular_z)
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': 'Control node not initialized'})

@app.route('/set_destination', methods=['POST'])
def set_destination():
    data = request.get_json()
    destination_id = data.get('destination_id', 0)
    
    if control_node:
        control_node.publish_destination(destination_id)
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': 'Control node not initialized'})

def main():
    global control_node
    
    # Инициализация ROS2
    rclpy.init()
    control_node = ControlNode()
    
    # Запуск ROS2 в отдельном потоке
    ros_thread = threading.Thread(target=lambda: rclpy.spin(control_node))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Запуск веб-сервера
    app.run(host='0.0.0.0', port=8080)
    
    # Очистка при завершении
    rclpy.shutdown()

if __name__ == '__main__':
    main()
