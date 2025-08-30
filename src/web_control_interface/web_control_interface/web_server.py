#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from flask import Flask, request, jsonify, send_from_directory
from flask_socketio import SocketIO
import threading

app = Flask(__name__, static_url_path='', static_folder='static')
socketio = SocketIO(app, async_mode='threading')
control_node = None

class ControlNode(Node):
    def __init__(self, socketio_instance):
        super().__init__('web_control_node')
        self.socketio = socketio_instance

        # --- Состояния для отправки на frontend ---
        self.robot_status = {
            "state": "INITIALIZING",
            "robot_lat": 0.0, "robot_lon": 0.0,
            "dest_lat": 0.0, "dest_lon": 0.0
        }

        # --- Издатели (Publishers) ---
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=1)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)
        
        dest_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=1)
        self.destination_pub_ = self.create_publisher(Int32, '/destination_id', dest_qos)

        # --- НОВЫЕ ПОДПИСЧИКИ (Subscribers) для получения статуса ---
        self.state_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        self.robot_gps_sub = self.create_subscription(
            NavSatFix, '/robot/gps/fix', self.robot_gps_callback, 10)
        self.dest_gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.dest_gps_callback, 10)
        
        self.get_logger().info('Web Control Node initialized publishers and subscribers.')

        # --- НОВЫЙ ТАЙМЕР для отправки данных через WebSocket ---
        # Отправляем данные на frontend 2 раза в секунду (2 Гц)
        self.status_update_timer = self.create_timer(0.5, self.emit_status_update)

    # --- Методы для публикации в ROS (остаются без изменений) ---
    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published to /cmd_vel: linear_x={linear_x}, angular_z={angular_z}')

    def publish_destination(self, destination_id):
        msg = Int32()
        msg.data = int(destination_id)
        self.destination_pub_.publish(msg)
        self.get_logger().info(f'Published to /destination_id: {msg.data}')

    # --- НОВЫЕ КОЛЛБЭКИ для подписчиков ---
    def status_callback(self, msg):
        self.robot_status["state"] = msg.data

    def robot_gps_callback(self, msg):
        self.robot_status["robot_lat"] = msg.latitude
        self.robot_status["robot_lon"] = msg.longitude

    def dest_gps_callback(self, msg):
        self.robot_status["dest_lat"] = msg.latitude
        self.robot_status["dest_lon"] = msg.longitude

    # --- НОВЫЙ МЕТОД для отправки данных через WebSocket (вызывается по таймеру) ---
    def emit_status_update(self):
        # Отправляем весь словарь self.robot_status всем подключенным клиентам
        self.socketio.emit('status_update', self.robot_status)
        self.get_logger().debug(f"Emitted status via WebSocket: {self.robot_status}")


# --- Маршруты Flask (остаются без изменений) ---
@app.route('/')
def index():
    return send_from_directory(app.static_folder, 'index.html')

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

# --- Основная функция main (остается почти без изменений) ---
def main(): 
    global control_node
    rclpy.init()
    # Передаем экземпляр socketio в узел ROS
    control_node = ControlNode(socketio)
    
    ros_thread = threading.Thread(target=lambda: rclpy.spin(control_node), daemon=True)
    ros_thread.start()
    
    try:
        # allow_unsafe_werkzeug=True может быть нужен для новых версий Flask-SocketIO
        socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True) 
    except Exception as e:
        if control_node: control_node.get_logger().error(f"Flask-SocketIO server error: {e}")
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()