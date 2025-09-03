#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from flask import Flask, request, jsonify, send_from_directory, send_file # <--- ДОБАВЛЕН send_file
from flask_socketio import SocketIO
import threading
import os
import io # <--- ДОБАВЛЕН io
from PIL import Image as PILImage # <--- ДОБАВЛЕНА библиотека Pillow

app = Flask(__name__, static_url_path='', static_folder='static')
socketio = SocketIO(app, async_mode='threading')
control_node = None

class ControlNode(Node):
    def __init__(self, socketio_instance):
        super().__init__('web_control_node')
        self.socketio = socketio_instance
        self.declare_parameter('maps_directory', '/root/ros2_ws/maps_storage')
        self.maps_directory = self.get_parameter('maps_directory').value
        self.get_logger().info(f"Serving maps from: {self.maps_directory}")

        self.robot_status = {
            "state": "INITIALIZING",
            "robot_lat": 0.0, "robot_lon": 0.0,
            "dest_lat": 0.0, "dest_lon": 0.0
        }

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=1)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)
        
        dest_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=1)
        self.destination_pub_ = self.create_publisher(Int32, '/destination_id', dest_qos)

        self.state_sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        self.robot_gps_sub = self.create_subscription(NavSatFix, '/robot/gps/current', self.robot_gps_callback, 10)
        self.dest_gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.dest_gps_callback, 10)
        
        self.get_logger().info('Web Control Node initialized publishers and subscribers.')
        self.status_update_timer = self.create_timer(0.5, self.emit_status_update)

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist(); msg.linear.x = float(linear_x); msg.angular.z = float(angular_z)
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published to /cmd_vel: linear_x={linear_x}, angular_z={angular_z}')

    def publish_destination(self, destination_id):
        msg = Int32(); msg.data = int(destination_id)
        self.destination_pub_.publish(msg)
        self.get_logger().info(f'Published to /destination_id: {msg.data}')

    def status_callback(self, msg): self.robot_status["state"] = msg.data
    def robot_gps_callback(self, msg):
        self.robot_status["robot_lat"] = msg.latitude
        self.robot_status["robot_lon"] = msg.longitude
    def dest_gps_callback(self, msg):
        self.robot_status["dest_lat"] = msg.latitude
        self.robot_status["dest_lon"] = msg.longitude

    def emit_status_update(self):
        self.socketio.emit('status_update', self.robot_status)
        self.get_logger().debug(f"Emitted status via WebSocket: {self.robot_status}")

@app.route('/')
def index(): return send_from_directory(app.static_folder, 'index.html')

@app.route('/control', methods=['POST'])
def control():
    data = request.get_json()
    if control_node:
        control_node.publish_velocity(data.get('linear_x', 0.0), data.get('angular_z', 0.0))
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error', 'message': 'ROS node not ready'}), 500

@app.route('/set_destination', methods=['POST'])
def set_destination():
    data = request.get_json()
    destination_id = data.get('destination_id')
    if destination_id is not None and control_node:
        control_node.publish_destination(destination_id)
        return jsonify({'status': 'success', 'message': f'Destination {destination_id} set.'})
    return jsonify({'status': 'error', 'message': 'Invalid request'}), 400

@app.route('/api/maps', methods=['GET'])
def get_maps():
    if not control_node: return jsonify({"error": "ROS node not ready"}), 500
    maps_dir = control_node.maps_directory
    if not os.path.isdir(maps_dir):
        control_node.get_logger().error(f"Maps directory '{maps_dir}' not found!")
        return jsonify({"error": "Maps directory not found on server"}), 500
    try:
        map_files = [os.path.splitext(f)[0] for f in os.listdir(maps_dir) if f.endswith('.yaml')]
        return jsonify(map_files)
    except Exception as e:
        control_node.get_logger().error(f"Error reading maps directory: {e}")
        return jsonify({"error": "Could not read maps directory"}), 500

# --- ИЗМЕНЕННЫЙ МАРШРУТ ДЛЯ ОТПРАВКИ ИЗОБРАЖЕНИЯ КАРТЫ ---
@app.route('/api/map_image/<map_name>', methods=['GET'])
def get_map_image(map_name):
    """Находит PGM файл, конвертирует его в PNG в памяти и отправляет браузеру."""
    if not control_node:
        return "ROS node not ready", 500
    
    maps_dir = control_node.maps_directory
    pgm_path = os.path.join(maps_dir, f"{map_name}.pgm")

    if not os.path.exists(pgm_path):
        control_node.get_logger().error(f"Map image file not found: {pgm_path}")
        return "Map image not found", 404

    try:
        # Открываем PGM с помощью Pillow и конвертируем в PNG в оперативной памяти
        with PILImage.open(pgm_path) as img:
            png_buffer = io.BytesIO()
            img.save(png_buffer, format="PNG")
            png_buffer.seek(0) # "Перематываем" буфер в начало
        
        # Отправляем данные из памяти, указывая тип содержимого
        return send_file(png_buffer, mimetype='image/png')

    except Exception as e:
        control_node.get_logger().error(f"Error converting map image to PNG: {e}")
        return "Internal server error", 500
# --- КОНЕЦ ИЗМЕНЕННОГО МАРШРУТА ---

def main(): 
    global control_node
    rclpy.init()
    control_node = ControlNode(socketio)
    ros_thread = threading.Thread(target=lambda: rclpy.spin(control_node), daemon=True)
    ros_thread.start()
    try:
        socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True) 
    except Exception as e:
        if control_node: control_node.get_logger().error(f"Flask-SocketIO server error: {e}")
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()