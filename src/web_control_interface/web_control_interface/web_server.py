#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
# --- ИМПОРТЫ QoS ---
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# --- КОНЕЦ ИМПОРТОВ QoS ---
from flask import Flask, request, jsonify, send_from_directory # Убрал render_template, если не используется
import threading
import os # os может понадобиться для работы с путями, но здесь не используется явно

# Убедитесь, что 'static' - это имя папки рядом с вашим скриптом, где лежит index.html
# или укажите абсолютный путь к папке static_folder
# Например: static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
# app = Flask(__name__, static_url_path='', static_folder=static_dir)
app = Flask(__name__, static_url_path='', static_folder='static') 
control_node = None

class ControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')

        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1 
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', cmd_vel_qos)
        
        # Для /destination_id можно оставить QoS по умолчанию или задать явно, если нужно
        # Например, reliable, если важно не потерять команду выбора цели
        dest_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, # или TRANSIENT_LOCAL, если хотите, чтобы новый подписчик получил последнее
            depth=1 
        )
        self.destination_pub_ = self.create_publisher(Int32, '/destination_id', dest_qos)
        self.get_logger().info('Web Control Node initialized, publishers created.')

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published to /cmd_vel: linear_x={linear_x}, angular_z={angular_z}')

    def publish_destination(self, destination_id):
        msg = Int32()
        msg.data = int(destination_id)
        self.destination_pub_.publish(msg)
        self.get_logger().info(f'Published to /destination_id: {msg.data}')

# --- ВОССТАНОВИТЕ ЭТИ МАРШРУТЫ FLASK ---
@app.route('/')
def index():
    # Убедитесь, что 'static' - это правильное имя папки,
    # и index.html находится непосредственно в ней.
    # Папка 'static' должна быть там, где Flask ее ищет (обычно рядом со скриптом или в корне приложения).
    self_dir = os.path.dirname(os.path.abspath(__file__))
    static_folder_path = os.path.join(self_dir, app.static_folder) # app.static_folder это 'static'
    
    if not os.path.isdir(static_folder_path):
        # Эта проверка не будет выполнена, если Flask не найдет папку static при старте
        # Но полезна для отладки, если бы мы формировали путь здесь
        if control_node:
             control_node.get_logger().error(f"Static folder '{static_folder_path}' not found!")
        else:
             print(f"Static folder '{static_folder_path}' not found!")
        return "Error: Static folder not found.", 404
    
    if not os.path.isfile(os.path.join(static_folder_path, 'index.html')):
        if control_node:
            control_node.get_logger().error(f"index.html not found in '{static_folder_path}'!")
        else:
            print(f"index.html not found in '{static_folder_path}'!")
        return "Error: index.html not found.", 404
            
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/control', methods=['POST'])
def control():
    data = request.get_json()
    linear_x = data.get('linear_x', 0.0)
    angular_z = data.get('angular_z', 0.0)
    
    if control_node and hasattr(control_node, 'publish_velocity'):
        control_node.publish_velocity(linear_x, angular_z)
        return jsonify({'status': 'success', 'message': 'Command sent.'})
    else:
        # Логируем ошибку на сервере, если узел не готов
        error_msg = 'Control node not initialized or publish_velocity not available'
        if control_node: control_node.get_logger().error(error_msg)
        else: print(f"ERROR: {error_msg}")
        return jsonify({'status': 'error', 'message': error_msg}), 500

@app.route('/set_destination', methods=['POST'])
def set_destination():
    data = request.get_json()
    destination_id = data.get('destination_id', None) # Получаем None, если ID не предоставлен
    
    if destination_id is None: # Проверка, что destination_id был в запросе
        if control_node: control_node.get_logger().warn("Received /set_destination request without destination_id.")
        else: print("WARN: Received /set_destination request without destination_id.")
        return jsonify({'status': 'error', 'message': 'destination_id missing'}), 400

    if control_node and hasattr(control_node, 'publish_destination'):
        control_node.publish_destination(destination_id)
        return jsonify({'status': 'success', 'message': f'Destination {destination_id} set.'})
    else:
        error_msg = 'Control node not initialized or publish_destination not available'
        if control_node: control_node.get_logger().error(error_msg)
        else: print(f"ERROR: {error_msg}")
        return jsonify({'status': 'error', 'message': error_msg}), 500
# --- КОНЕЦ ВОССТАНОВЛЕННЫХ МАРШРУТОВ ---

def main(): 
    global control_node
    rclpy.init()
    control_node = ControlNode() # Создаем узел
    
    # Запускаем ROS node в отдельном потоке
    ros_thread = threading.Thread(target=lambda: rclpy.spin(control_node), daemon=True)
    ros_thread.start()
    
    # Запускаем Flask сервер
    # debug=True полезно для разработки, но выключайте для "продакшена"
    # use_reloader=False может быть полезно, если возникают проблемы с двойной инициализацией узла ROS
    try:
        # app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)
        app.run(host='0.0.0.0', port=8080) # Убедитесь, что порт не занят
    except SystemExit: # SystemExit может быть вызван Ctrl+C во Flask, если он не в daemon потоке
        if control_node: control_node.get_logger().info("Flask server shutting down (SystemExit).")
    except Exception as e:
        if control_node:
            control_node.get_logger().error(f"Flask server critical error: {e}")
        else:
            print(f"Flask server critical error: {e}") # Если узел ROS еще не создан
    finally:
        if control_node: 
            control_node.get_logger().info("Attempting to shutdown web_control_node and rclpy.")
        
        # rclpy.shutdown() должен корректно остановить spin и позволить узлу уничтожиться,
        # если поток был daemon. Если нет, может потребоваться более явное управление.
        if rclpy.ok():
            rclpy.shutdown()
        
        if ros_thread.is_alive():
            # Попытка дождаться завершения потока ROS, если он не daemon или shutdown не сработал сразу
            # Это опционально и зависит от того, как быстро вы хотите завершать приложение
            ros_thread.join(timeout=1.0) 
            if ros_thread.is_alive() and control_node:
                 control_node.get_logger().warn("ROS thread did not exit cleanly.")


if __name__ == '__main__':
    main()