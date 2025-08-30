#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Для подписки на курс от компаса
from std_msgs.msg import Int32, Bool
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, LaserScan # LaserScan добавлен
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist # PoseStamped может понадобиться для Nav2
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import yaml
import os
import math
import time
from nav2_msgs.srv import SaveMap
# import sys # Не используется, можно удалить

class NavigationManagerNode(Node):

    def __init__(self):
            super().__init__('navigation_manager_node')

            # --- Параметры ---
            self.declare_parameter('config_file', '/root/ros2_ws/src/robot_navigation_manager/config/destinations.yaml')
            self.declare_parameter('maps_directory', '/root/ros2_ws/maps_storage')
            self.declare_parameter('time_at_destination_duration', 10.0)
            self.declare_parameter('gps_logging_interval', 5.0)
            self.declare_parameter('gps_nav_tolerance_meters', 2.0)
            self.declare_parameter('gps_obstacle_detection_dist_meters', 0.7)
            self.declare_parameter('gps_forward_speed', 0.2)
            self.declare_parameter('gps_turn_speed', 0.3)
            self.declare_parameter('gps_control_frequency_hz', 5.0)
            self.declare_parameter('gps_wait_timeout_duration', 15.0)
            
            # --- НОВЫЕ ПАРАМЕТРЫ ДЛЯ ПОСТОЯННОЙ ПРОВЕРКИ GPS ---
            self.declare_parameter('hw_gps_timeout_sec', 15.0) # Сколько секунд ждать аппаратный GPS, прежде чем переключиться на MQTT
            self.declare_parameter('gps_source_check_period_sec', 3.0) # Как часто проверять состояние источника GPS

            # --- Состояния и переменные ---
            # Существующие
            self.current_state = 'IDLE'
            self.current_destination_id = None
            self.current_gps_location = None   # NavSatFix - ОБЩЕЕ МЕСТОПОЛОЖЕНИЕ РОБОТА
            self.dynamic_gps_target = None     # NavSatFix (Для цели 1 из /gps/fix)
            self.current_map_pose = None       # geometry_msgs/Pose (из AMCL)
            self.current_true_heading_rad = None # Курс от компаса
            self.current_scan = None
            self.target_gps_for_direct_nav = None
            self.gps_navigation_timer = None
            self.active_nav2_goal_handle = None
            self.time_at_destination_timer = None
            self.waiting_for_gps_timeout_timer = None
            self.pending_destination_info_for_gps_wait = None
            self.current_task_specific_map_exists = False
            self.current_task_map_path = ""
            self.perform_mapping_for_current_task = False

            self.destinations = {}
            self.base_location = None
            self.map_areas = {}
            
            # --- ИЗМЕНЕННЫЕ ПЕРЕМЕННЫЕ СОСТОЯНИЯ ДЛЯ ВЫБОРА GPS ---
            self.active_gps_source = 'UNKNOWN' # 'UNKNOWN', 'HARDWARE', 'MQTT_FALLBACK'
            self.last_hw_gps_time = None # Время последнего получения данных от аппаратного GPS

            # --- Конфигурация ---
            self.load_configuration()

            # --- Получение значений параметров ---
            self.time_at_destination_duration = self.get_parameter('time_at_destination_duration').value
            self.gps_logging_interval = self.get_parameter('gps_logging_interval').value
            self.gps_nav_tolerance = self.get_parameter('gps_nav_tolerance_meters').value
            self.obstacle_detection_distance = self.get_parameter('gps_obstacle_detection_dist_meters').value
            self.forward_speed_gps = self.get_parameter('gps_forward_speed').value
            self.turn_speed_gps = self.get_parameter('gps_turn_speed').value
            gps_control_frequency = self.get_parameter('gps_control_frequency_hz').value
            self.gps_timer_period = 1.0 / gps_control_frequency if gps_control_frequency > 0 else 0.2
            self.gps_wait_timeout_duration = self.get_parameter('gps_wait_timeout_duration').value
            self.maps_directory = self.get_parameter('maps_directory').value
            os.makedirs(self.maps_directory, exist_ok=True)
            self.hw_gps_timeout_duration = self.get_parameter('hw_gps_timeout_sec').value
            self.gps_source_check_period = self.get_parameter('gps_source_check_period_sec').value
            self.get_logger().info(f"GPS source status will be checked every {self.gps_source_check_period}s with a timeout of {self.hw_gps_timeout_duration}s.")

            # --- Подписки ---
            self.destination_sub = self.create_subscription(Int32, '/destination_id', self.destination_callback, 10)
            self.dynamic_gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_fix_callback, 10)
            self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
            self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)
            self.heading_sub = self.create_subscription(Float32, '/robot/heading', self.heading_callback, 10)
            
            # --- ИЗМЕНЕНИЕ: Два подписчика на GPS робота ---
            self.robot_hw_gps_sub = self.create_subscription(NavSatFix, '/robot/gps/fix', self.robot_hw_gps_callback, 10)
            self.robot_mqtt_gps_sub = self.create_subscription(NavSatFix, '/robot/gps/fix_mqtt', self.robot_mqtt_gps_callback, 10)
            self.get_logger().info("Subscribed to all topics including '/robot/gps/fix' (Hardware) and '/robot/gps/fix_mqtt' (MQTT Fallback)")

            # --- Паблишеры и клиенты ---
            self.gps_target_pub = self.create_publisher(NavSatFix, '/gps_target', 10)
            self.gps_mode_control_pub = self.create_publisher(Bool, '/gps_mode_control', 10)
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
            self.get_logger().info("Publishers and action/service clients created.")
            self.state_publisher = self.create_publisher(String, '/robot_status', 10)

            # --- НОВЫЙ ТАЙМЕР ДЛЯ ПОСТОЯННОЙ ПРОВЕРКИ ИСТОЧНИКА GPS ---
            self.gps_source_check_timer = self.create_timer(self.gps_source_check_period, self._check_gps_source_status)
            
            # --- Финальная инициализация ---
            self.set_state('IDLE')
            self.get_logger().info('Navigation Manager Node initialized.')

    def load_configuration(self):
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.get_logger().info(f"Attempting to load configuration from: {config_file_path}")

        if not os.path.exists(config_file_path):
            self.get_logger().error(f"Configuration file NOT FOUND at '{config_file_path}'. Navigation will be severely limited.")
            # Устанавливаем пустые значения по умолчанию, если файл не найден
            self.destinations = {}
            self.base_location = None
            self.map_areas = {}
            return

        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info("Configuration file loaded successfully.")

            self.destinations = config.get('destinations', {})
            self.get_logger().info(f"Loaded {len(self.destinations)} destinations.")

            self.base_location = config.get('base', None)
            if self.base_location:
                self.get_logger().info("Base location configured.")
            else:
                self.get_logger().warn("No 'base' location found in config.")

            self.map_areas = config.get('map_areas', {})
            self.get_logger().info(f"Loaded {len(self.map_areas)} map areas.")

            if 'time_at_destination_duration' in config and isinstance(config['time_at_destination_duration'], (int, float)):
                duration_from_config = float(config['time_at_destination_duration'])
                # Обновляем параметр ROS 2, если он был объявлен
                if self.has_parameter('time_at_destination_duration'):
                    self.set_parameters([rclpy.Parameter('time_at_destination_duration', rclpy.Parameter.Type.DOUBLE, duration_from_config)])
                    # self.time_at_destination_duration = duration_from_config # Обновится при следующем get_parameter или присвоить здесь явно
                    self.get_logger().info(f"Parameter 'time_at_destination_duration' will be updated to {duration_from_config}s from config file.")
                else: # На случай если параметр не был объявлен по какой-то причине
                     self.time_at_destination_duration = duration_from_config


        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML configuration file: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse configuration file: {e}")
            self.destinations = {}
            self.base_location = None
            self.map_areas = {}

    # --- Callback Methods ---

    def _gps_wait_timeout_callback(self):
        self.get_logger().warn(f"Timeout waiting for dynamic GPS data for Destination ID {self.current_destination_id} "
                               f"(target topic: {self.pending_destination_info_for_gps_wait.get('topic_name', 'N/A') if self.pending_destination_info_for_gps_wait else 'N/A'}).")

        if self.waiting_for_gps_timeout_timer: # На всякий случай, хотя он должен быть этим таймером
            self.waiting_for_gps_timeout_timer.cancel()
            self.waiting_for_gps_timeout_timer = None

        self.pending_destination_info_for_gps_wait = None

        # Если мы все еще ждали, значит навигация не началась, переходим в IDLE
        if self.current_state == 'WAITING_FOR_DYNAMIC_GPS':
            self.get_logger().info("Returning to IDLE due to GPS wait timeout.")
            self.stop_navigation() # stop_navigation сбросит current_destination_id и состояние
        else:
            self.get_logger().info(f"GPS wait timeout occurred, but current state is {self.current_state}, not WAITING_FOR_DYNAMIC_GPS. No state change by timeout.")


    def heading_callback(self, msg: Float32):
        self.current_true_heading_rad = msg.data
        # Логирование можно добавить для отладки:
        self.get_logger().debug(f"Received compass heading: {math.degrees(self.current_true_heading_rad):.1f} degrees")

    def robot_gps_callback(self, msg):
        self.current_gps_location = msg
        current_time = time.time()
        # Логирование с троттлингом для уменьшения спама
        if self.gps_logging_interval > 0 and (current_time - self.last_gps_log_time > self.gps_logging_interval):
            self.get_logger().info(f"Robot GPS: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}")
            self.last_gps_log_time = current_time

    def gps_fix_callback(self, msg): # Для динамической цели 1 (телефон)
        # Предполагаем, что destination '1' использует этот топик, если source='topic'
        # В более общем случае, нужно будет проверять self.current_destination_id
        # и соответствие topic_name из конфигурации.
        dest1_config = self.destinations.get('1', {})
        if dest1_config.get('source') == 'topic' and dest1_config.get('topic_name') == '/gps/fix':
            self.dynamic_gps_target = msg
            # Логирование с троттлингом можно добавить аналогично robot_gps_callback
            # self.get_logger().info(f"Dynamic GPS target 1 updated: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}", throttle_duration_sec=self.gps_logging_interval)
            self.get_logger().debug(f"Dynamic GPS target 1 updated: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}", 
                               throttle_duration_sec=self.gps_logging_interval if self.gps_logging_interval > 0 else 1.0)

            # Если мы ожидали эти данные для текущей цели
        if self.current_state == 'WAITING_FOR_DYNAMIC_GPS' and \
           self.current_destination_id == 1 and \
           self.pending_destination_info_for_gps_wait is not None:

            self.get_logger().info(f"Dynamic GPS for Destination {self.current_destination_id} (from /gps/fix) "
                                   "received while waiting. Proceeding with navigation planning.")

            if self.waiting_for_gps_timeout_timer: # Отменяем таймер ожидания
                self.waiting_for_gps_timeout_timer.cancel()
                self.waiting_for_gps_timeout_timer = None

            stored_info = self.pending_destination_info_for_gps_wait
            self.pending_destination_info_for_gps_wait = None # Очищаем

            # Не меняем состояние здесь, start_navigation_to_destination сделает это.
            # self.set_state('PLANNING_NAVIGATION') # Это произойдет внутри start_navigation_to_destination
            self.start_navigation_to_destination(stored_info) # Повторно пытаемся начать навигацию


    def amcl_pose_callback(self, msg):
        self.current_map_pose = msg.pose.pose # Извлекаем Pose из PoseWithCovarianceStamped
        # Логирование с троттлингом можно добавить аналогично robot_gps_callback
        # current_time = time.time()
        # if self.gps_logging_interval > 0 and (current_time - getattr(self, 'last_amcl_log_time', 0) > self.gps_logging_interval):
        #    self.get_logger().info(f"AMCL Pose: x {self.current_map_pose.position.x:.2f}, y {self.current_map_pose.position.y:.2f}")
        #    self.last_amcl_log_time = current_time


    def scan_callback(self, msg):
        self.current_scan = msg

    def destination_callback(self, msg):
        destination_id_int = msg.data
        self.get_logger().info(f"Received new destination ID: {destination_id_int}")

        if self.current_state != 'IDLE' and self.current_state != 'AT_DESTINATION':
             self.get_logger().warn(f"Received new destination while robot is busy in state '{self.current_state}'. Stopping current task first.")
             self.stop_navigation() # Важно остановить текущую задачу перед новой

        if destination_id_int == 0: # ID 0 для отмены/остановки
            self.get_logger().info("Destination ID 0 received: Commanding stop and return to IDLE state.")
            self.stop_navigation() # stop_navigation() уже установит IDLE
            return

        dest_key = str(destination_id_int)
        if dest_key not in self.destinations:
            self.get_logger().warn(f"Destination ID {destination_id_int} not found in configuration. Returning to IDLE.")
            self.set_state('IDLE') # Убедимся, что состояние IDLE
            return
    
        destination_info = self.destinations[dest_key]
        self.current_destination_id = destination_id_int # Сохраняем числовой ID
        self.set_state('PLANNING_NAVIGATION')
        self.start_navigation_to_destination(destination_info)
    
    # --- НОВЫЕ И ИЗМЕНЕННЫЕ КОЛЛБЭКИ ДЛЯ ВЫБОРА ИСТОЧНИКА GPS ---

    def robot_hw_gps_callback(self, msg: NavSatFix):
        """Callback для данных от аппаратного GPS (/robot/gps/fix)."""
        # Просто обновляем время последнего получения сигнала
        self.last_hw_gps_time = self.get_clock().now()
        
        # Если аппаратный GPS является активным источником, обновляем текущее положение робота
        if self.active_gps_source == 'HARDWARE':
            self.current_gps_location = msg
            # Логирование с троттлингом для уменьшения спама в консоли
            self.get_logger().info(f"Robot GPS (HW ACTIVE): Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}",
                                   throttle_duration_sec=self.gps_logging_interval)

    def robot_mqtt_gps_callback(self, msg: NavSatFix):
        """Callback для данных от MQTT GPS (/robot/gps/fix_mqtt)."""
        # MQTT GPS используется только в режиме отката (fallback)
        if self.active_gps_source == 'MQTT_FALLBACK':
            self.current_gps_location = msg
            self.get_logger().info(f"Robot GPS (MQTT FALLBACK): Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}",
                                   throttle_duration_sec=self.gps_logging_interval)
    
    def _check_gps_source_status(self):
        """Периодически проверяет, какой источник GPS использовать. Вызывается по таймеру."""
        now = self.get_clock().now()
        
        # Проверяем, есть ли у нас вообще сигнал от аппаратного GPS
        if self.last_hw_gps_time is None:
            # Сигнала от аппаратного GPS еще ни разу не было
            if self.active_gps_source != 'MQTT_FALLBACK':
                self.get_logger().warn("No signal from hardware GPS since startup. Using MQTT GPS as fallback.")
                self.active_gps_source = 'MQTT_FALLBACK'
            return # Выходим, ждем дальше

        # Если сигнал был, проверяем, как давно
        time_since_last_hw_gps = (now - self.last_hw_gps_time).nanoseconds / 1e9

        if time_since_last_hw_gps <= self.hw_gps_timeout_duration:
            # Аппаратный GPS работает, используем его
            if self.active_gps_source != 'HARDWARE':
                self.get_logger().info(f"Hardware GPS signal is active (last seen {time_since_last_hw_gps:.1f}s ago). Switching to HARDWARE source.")
                self.active_gps_source = 'HARDWARE'
        else:
            # Сигнал от аппаратного GPS пропал
            if self.active_gps_source != 'MQTT_FALLBACK':
                self.get_logger().warn(f"Hardware GPS signal lost for {time_since_last_hw_gps:.1f}s. "
                                       f"Falling back to MQTT GPS.")
                self.active_gps_source = 'MQTT_FALLBACK'
    
    # --- Navigation Logic Methods ---
    def set_state(self, new_state):
        if self.current_state != new_state:
            self.get_logger().info(f"State transition: {self.current_state} -> {new_state}")
            self.current_state = new_state
            state_msg = String()
            state_msg.data = self.current_state
            self.state_publisher.publish(state_msg)

    def _determine_navigation_parameters(self, target_location_info, is_base_return=False):
        """
        Вспомогательный метод для определения параметров навигации (карта/GPS, координаты).
        target_location_info: dict с информацией о цели (из self.destinations или self.base_location).
        is_base_return: bool флаг, если это возврат на базу.
        Возвращает: (can_navigate, use_map_nav, target_map_pose_stamped, target_gps_navsatfix)
        target_map_pose_stamped: geometry_msgs/PoseStamped или None
        target_gps_navsatfix: sensor_msgs/NavSatFix или None
        """
        target_gps_navsatfix = None
        target_map_pose_stamped = None
        use_map_nav = False
        can_navigate = False

        if not target_location_info:
            self.get_logger().error("Target location info is None. Cannot determine navigation parameters.")
            return False, False, None, None
            
        source_type = target_location_info.get('source', 'recorded')
        target_name = target_location_info.get('name', 'Unnamed Target')

        self.get_logger().info(f"Determining nav params for '{target_name}', source: '{source_type}'")
            # --- НОВЫЙ БЛОК: Принудительный GPS-режим, если активно картографирование ---
        if self.perform_mapping_for_current_task and not is_base_return: # Не применяем для возврата на базу, если только база не в новой зоне
            self.get_logger().info(f"Mapping mode is active for '{target_name}'. Forcing GPS navigation for exploration.")
            use_map_nav = False # Не используем карту, даже если она указана в destination_info как общая
            target_map_pose_stamped = None

            # Пытаемся получить GPS координаты для цели
            if target_location_info.get('source') == 'topic':
                # Логика для динамической цели (например, Цель 1)
                expected_topic = target_location_info.get('topic_name')
                if self.current_destination_id == 1 and expected_topic == '/gps/fix' and self.dynamic_gps_target:
                    target_gps_navsatfix = self.dynamic_gps_target
                    can_navigate = True
                else:
                    self.get_logger().warn(f"'{target_name}' (mapping mode): Dynamic GPS data not available.")
                    can_navigate = False
            elif 'gps_coords' in target_location_info: # Для 'recorded' целей берем их gps_coords
                coords = target_location_info['gps_coords']
                if isinstance(coords, dict) and 'latitude' in coords and 'longitude' in coords:
                    target_gps_navsatfix = NavSatFix()
                    target_gps_navsatfix.latitude = float(coords['latitude'])
                    target_gps_navsatfix.longitude = float(coords['longitude'])
                    target_gps_navsatfix.altitude = float(coords.get('altitude', 0.0))
                    can_navigate = True
                else:
                    self.get_logger().error(f"'{target_name}' (mapping mode): Recorded 'gps_coords' are invalid.")
                    can_navigate = False
            else:
                self.get_logger().error(f"'{target_name}' (mapping mode): No GPS coordinates available for exploration.")
                can_navigate = False

            return can_navigate, use_map_nav, target_map_pose_stamped, target_gps_navsatfix
        # --- КОНЕЦ НОВОГО БЛОКА ---
        if source_type == 'topic':
            # Пока что жестко привязано к self.current_destination_id == 1 для /gps/fix
            # и self.dynamic_gps_target. В будущем это нужно обобщить.
            expected_topic = target_location_info.get('topic_name')
            if self.current_destination_id == 1 and expected_topic == '/gps/fix' and self.dynamic_gps_target:
                target_gps_navsatfix = self.dynamic_gps_target
                self.get_logger().info(f"Using dynamic GPS target from topic '{expected_topic}' for '{target_name}' (Dest ID: {self.current_destination_id})")
                can_navigate = True
            else:
                self.get_logger().warn(f"'{target_name}': Source is 'topic' ('{expected_topic}'), but no valid dynamic GPS data available "
                                       f"(current_dest_id: {self.current_destination_id}, dynamic_gps_target is set: {self.dynamic_gps_target is not None}).")
        
        elif source_type == 'recorded':
            map_name_cfg = target_location_info.get('map_name')
            map_target_pose_dict = target_location_info.get('map_target_pose') # Это dict из YAML

            # Попытка навигации по карте, если робот находится в зоне карты
            if map_name_cfg and map_target_pose_dict and self.current_gps_location:
                map_area_details = self.map_areas.get(map_name_cfg)
                if map_area_details and 'bounds' in map_area_details:
                    bounds = map_area_details['bounds']
                    # Проверяем, находится ли РОБОТ в пределах GPS-границ известной карты
                    if (bounds.get('min_lat', -91) <= self.current_gps_location.latitude <= bounds.get('max_lat', 91) and
                        bounds.get('min_lon', -181) <= self.current_gps_location.longitude <= bounds.get('max_lon', 181)):
                        
                        # Конвертируем YAML/dict в PoseStamped
                        ps = PoseStamped()
                        ps.header.frame_id = map_target_pose_dict.get('header', {}).get('frame_id', 'map')
                        ps.header.stamp = self.get_clock().now().to_msg()
                        pose_cfg = map_target_pose_dict.get('pose', {})
                        pos_cfg = pose_cfg.get('position', {})
                        orient_cfg = pose_cfg.get('orientation', {})

                        ps.pose.position.x = float(pos_cfg.get('x', 0.0))
                        ps.pose.position.y = float(pos_cfg.get('y', 0.0))
                        ps.pose.position.z = float(pos_cfg.get('z', 0.0)) # Обычно 0 для 2D навигации
                        ps.pose.orientation.x = float(orient_cfg.get('x', 0.0))
                        ps.pose.orientation.y = float(orient_cfg.get('y', 0.0))
                        ps.pose.orientation.z = float(orient_cfg.get('z', 0.0))
                        ps.pose.orientation.w = float(orient_cfg.get('w', 1.0)) # Важно для кватерниона по умолчанию

                        target_map_pose_stamped = ps
                        use_map_nav = True
                        can_navigate = True
                        self.get_logger().info(f"'{target_name}': Will use map navigation in area '{map_name_cfg}'. Robot is within map bounds.")
                    else:
                        self.get_logger().info(f"'{target_name}': Robot (at {self.current_gps_location.latitude:.5f}, {self.current_gps_location.longitude:.5f}) "
                                               f"is outside map area '{map_name_cfg}' bounds. Will try GPS if available.")
                else:
                    self.get_logger().warn(f"'{target_name}': Map area '{map_name_cfg}' not found in 'map_areas' or lacks 'bounds' definition.")

            # Если навигация по карте не выбрана/невозможна, пробуем GPS из конфига
            if not use_map_nav and 'gps_coords' in target_location_info:
                coords = target_location_info['gps_coords']
                if isinstance(coords, dict) and 'latitude' in coords and 'longitude' in coords:
                    target_gps_navsatfix = NavSatFix()
                    target_gps_navsatfix.latitude = float(coords['latitude'])
                    target_gps_navsatfix.longitude = float(coords['longitude'])
                    target_gps_navsatfix.altitude = float(coords.get('altitude', 0.0)) 
                    can_navigate = True # Может быть уже True, если map_nav не удался, но GPS есть
                    self.get_logger().info(f"'{target_name}': Using recorded GPS coordinates.")
                else:
                    self.get_logger().error(f"'{target_name}': Recorded 'gps_coords' are invalid or missing lat/lon.")
            elif not use_map_nav and not target_gps_navsatfix: # Если и не карта, и нет GPS из конфига
                 self.get_logger().error(f"'{target_name}': Lacks valid map_target_pose (or robot outside map bounds) AND lacks gps_coords for 'recorded' source.")
        else:
            self.get_logger().error(f"'{target_name}': Unknown source type '{source_type}'.")

        return can_navigate, use_map_nav, target_map_pose_stamped, target_gps_navsatfix


    def start_navigation_to_destination(self, destination_info):
        # destination_info это dict для текущей цели из self.destinations
        target_name = destination_info.get('name', f"Destination ID {self.current_destination_id}")
        self.get_logger().info(f"Processing navigation request for '{target_name}'.")
        self.current_task_specific_map_exists = False # Флаг для текущей задачи
        self.current_task_map_path = "" # Путь к карте для текущей задачи, если она есть/будет
        
        if self.current_destination_id is not None and self.current_destination_id > 0: # Для Destination 0 (стоп) карта не нужна
            # Формируем ожидаемое имя файла карты для текущего пункта назначения
            # Например, для Destination 1 будет destmap1.yaml, для Destination 2 - destmap2.yaml
            map_file_name = f"destmap{self.current_destination_id}.yaml"
            self.current_task_map_path = os.path.join(self.maps_directory, map_file_name)

            if os.path.exists(self.current_task_map_path):
                self.current_task_specific_map_exists = True
                self.get_logger().info(f"Map for '{target_name}' found at: {self.current_task_map_path}")
            else:
                self.current_task_specific_map_exists = False
                self.get_logger().info(f"Map for '{target_name}' NOT found at: {self.current_task_map_path}. SLAM mode may be required.")
                 # --- НОВОЕ: Устанавливаем флаг картографирования ---
                self.perform_mapping_for_current_task = True
                self.get_logger().info(f"SLAM mode activated for this task. Will navigate via GPS to build map: {map_file_name}")
        else: # Для ID=0 (стоп) или если ID не установлен, картографирование не выполняем
            self.perform_mapping_for_current_task = False
        # --- КОНЕЦ НОВОГО БЛОКА И БЛОКА ПРОВЕРКИ КАРТЫ ---
        source_type = destination_info.get('source')
        expected_topic = destination_info.get('topic_name')

                # --- НАЧАЛО ИСПРАВЛЕННОЙ ЛОГИКИ ---
        # Проверка для динамических GPS целей (пока только Цель 1), если данные еще не пришли
        if source_type == 'topic' and \
           self.current_destination_id == 1 and \
           expected_topic == '/gps/fix' and \
           self.dynamic_gps_target is None:
            
            # Этот блок выполняется, ТОЛЬКО ЕСЛИ УСЛОВИЕ ВЫШЕ ИСТИННО
            if self.current_state == 'WAITING_FOR_DYNAMIC_GPS' and \
               self.pending_destination_info_for_gps_wait is not None and \
               self.pending_destination_info_for_gps_wait.get('name') == destination_info.get('name'): # Проверяем, что ждем ту же цель
                self.get_logger().info(f"Already waiting for GPS for '{target_name}'. Current wait continues.")
                return # Выходим, так как уже в процессе ожидания для этой цели

            self.set_state('WAITING_FOR_DYNAMIC_GPS')
            self.pending_destination_info_for_gps_wait = destination_info
            
            if self.waiting_for_gps_timeout_timer: # Отменяем предыдущий таймер ожидания, если он был активен
                self.waiting_for_gps_timeout_timer.cancel()
                self.waiting_for_gps_timeout_timer = None # Сбрасываем ссылку на таймер
            
            self.waiting_for_gps_timeout_timer = self.create_timer(
                self.gps_wait_timeout_duration,
                self._gps_wait_timeout_callback 
            )
            self.get_logger().info(f"No dynamic GPS data yet for '{target_name}'. Waiting for data on '{expected_topic}' "
                                   f"for up to {self.gps_wait_timeout_duration} seconds.")
            return # Выходим из функции, будем ждать данных в gps_fix_callback или таймаута
        
        # Если мы дошли сюда, значит:
        # 1. Это не динамическая GPS-цель, для которой нужно ждать данные (например, source 'recorded'), ИЛИ
        # 2. Это динамическая GPS-цель (Цель 1), но данные для нее УЖЕ ЕСТЬ (self.dynamic_gps_target is NOT None).
        
        # Теперь определяем параметры навигации с учетом текущей доступности данных
        can_navigate, use_map_nav, target_map_pose, target_gps_navsatfix = \
            self._determine_navigation_parameters(destination_info, is_base_return=False)
        
        # --- КОНЕЦ ИСПРАВЛЕННОЙ ЛОГИКИ (начальная часть) ---

        if can_navigate:
            # Отменяем предыдущую активную цель Nav2, если она была
            if self.active_nav2_goal_handle:
                self.get_logger().info("Cancelling previous active Nav2 goal before starting new navigation.")
                self.active_nav2_goal_handle.cancel_goal_async()
                self.active_nav2_goal_handle = None

            if use_map_nav and target_map_pose:
                self.set_state('NAVIGATING_MAP')
                self.send_nav2_goal(target_map_pose)
            elif target_gps_navsatfix:
                self.target_gps_for_direct_nav = target_gps_navsatfix
                self.set_state('NAVIGATING_GPS')
                self._start_gps_direct_navigation_loop()
            else:
                self.get_logger().error(f"'{target_name}': Navigation determined possible, but no valid map_pose or gps_navsatfix obtained. Stopping.")
                self.stop_navigation()
        else:
            self.get_logger().warn(f"Cannot navigate to '{target_name}'. Insufficient data or configuration. Returning to IDLE.")
            self.stop_navigation() # stop_navigation установит IDLE

    def send_nav2_goal(self, pose_stamped: PoseStamped): # Явно указываем тип
        if pose_stamped is None:
            self.get_logger().error("Cannot send Nav2 goal - pose_stamped is None.")
            self.stop_navigation() # Переходим в IDLE и останавливаем все
            return

        if not self._nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server "navigate_to_pose" not available. Check if Nav2 is running.')
            self.stop_navigation()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped # NavigateToPose.Goal ожидает PoseStamped

        self.get_logger().info(f"Sending goal to Nav2: Position(x={pose_stamped.pose.position.x:.2f}, y={pose_stamped.pose.position.y:.2f}), "
                               f"Frame='{pose_stamped.header.frame_id}'")
        
        self._send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        # Добавляем обработчик ответа на отправку цели
        self._send_goal_future.add_done_callback(self.nav2_goal_response_callback)


    def nav2_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Exception while sending Nav2 goal: {e}")
            self.active_nav2_goal_handle = None # Убедимся, что сброшен
            self.stop_navigation()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal was REJECTED by the server.')
            self.active_nav2_goal_handle = None
            self.stop_navigation()
            return

        self.get_logger().info('Nav2 goal ACCEPTED by the server.')
        self.active_nav2_goal_handle = goal_handle # Сохраняем для возможной отмены
        
        self.get_logger().info('Getting Nav2 result async...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav2_get_result_callback)


    def nav2_get_result_callback(self, future):
        # Сначала сбрасываем handle, так как задача завершена (успешно, с ошибкой или отменена)
        # active_nav2_goal_handle должен быть None, если мы не в середине активной задачи Nav2.
        # Если goal_handle был сохранен, значит это был он.
        # current_active_handle = self.active_nav2_goal_handle # Для сравнения, если нужно
        self.active_nav2_goal_handle = None 

        try:
            result_wrapper = future.result() # Это GetResult.Response, который содержит status и result
        except Exception as e:
            self.get_logger().error(f"Exception while getting Nav2 result: {e}")
            self.stop_navigation() # Переход в IDLE
            return
            
        status = result_wrapper.status
        # result = result_wrapper.result # NavigateToPose.Result пустой

        # Импорт здесь, чтобы не засорять начало файла, или вынести в глобальные, если используется часто
        from action_msgs.msg import GoalStatus 

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 navigation SUCCEEDED.")
            if self.current_state == 'NAVIGATING_MAP':
                self.set_state('AT_DESTINATION')
                self.start_time_at_destination_timer()
            elif self.current_state == 'RETURNING_TO_BASE_MAP':
                self.get_logger().info("Successfully returned to base (map navigation). Processing task completion.")
                self.active_nav2_goal_handle = None # Сбрасываем хэндл цели Nav2
                self._process_task_completion() # <--- НОВЫЙ ВЫЗОВ
                # self.set_state('IDLE') и сброс current_destination_id теперь произойдет внутри _process_task_completion -> stop_navigation
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Nav2 navigation was CANCELED.")
            # stop_navigation() уже был вызван, если отмена была инициирована нами.
            # Если отмена пришла от сервера или другого источника, переходим в IDLE.
            if self.current_state not in ['IDLE']: # Если мы еще не в IDLE из-за stop_navigation
                self.set_state('IDLE')
        else: # ABORTED, UNKNOWN, etc.
            self.get_logger().error(f"Nav2 navigation FAILED with status: {GoalStatus.GoalStatus.debug_string(status)} ({status})")
            self.stop_navigation() # Переход в IDLE и остановка всего

    def start_time_at_destination_timer(self):
        self.get_logger().info(f"Robot is AT_DESTINATION. Starting timer for {self.time_at_destination_duration:.1f} seconds.")
        if self.time_at_destination_timer is not None:
            self.time_at_destination_timer.cancel() # Отменяем предыдущий, если был
        
        self.time_at_destination_timer = self.create_timer(
            self.time_at_destination_duration, 
            self.time_at_destination_timer_callback
        )

    def time_at_destination_timer_callback(self):
        self.get_logger().info("Time at destination expired.")
        if self.time_at_destination_timer is not None:
            self.time_at_destination_timer.cancel()
            self.time_at_destination_timer = None
        
        if self.current_state == 'AT_DESTINATION': # Убедимся, что мы все еще в этом состоянии
            self.get_logger().info("Initiating return to base from AT_DESTINATION.")
            self.return_to_base()
        else:
            self.get_logger().warn(f"Timer callback executed but current state is '{self.current_state}', not 'AT_DESTINATION'. Not returning to base automatically.")

    def return_to_base(self):
        self.get_logger().info("Processing 'return_to_base' request.")
        if not self.base_location:
            self.get_logger().error("Base location not configured. Cannot return to base.")
            self.stop_navigation() # Установит IDLE
            return
        
        target_name = self.base_location.get('name', "Base")
        self.get_logger().info(f"Attempting to return to '{target_name}'.")

        # Сохраняем и временно сбрасываем current_destination_id для _determine_navigation_parameters,
        # если логика выбора цели зависит от ID (например, для 'topic' source). Для базы это обычно 'recorded'.
        # original_destination_id = self.current_destination_id
        # self.current_destination_id = None # Или специальный ID для базы

        can_navigate, use_map_nav, target_map_pose, target_gps_navsatfix = \
            self._determine_navigation_parameters(self.base_location, is_base_return=True)
        
        # self.current_destination_id = original_destination_id # Восстанавливаем, если нужно

        if can_navigate:
            # Отменяем предыдущую активную цель Nav2, если она была
            if self.active_nav2_goal_handle:
                self.get_logger().info("Cancelling previous active Nav2 goal before returning to base.")
                self.active_nav2_goal_handle.cancel_goal_async()
                self.active_nav2_goal_handle = None

            if use_map_nav and target_map_pose:
                self.set_state('RETURNING_TO_BASE_MAP')
                self.send_nav2_goal(target_map_pose)
            elif target_gps_navsatfix:
                self.target_gps_for_direct_nav = target_gps_navsatfix
                self.set_state('RETURNING_TO_BASE_GPS')
                self._start_gps_direct_navigation_loop()
            else:
                self.get_logger().error(f"'{target_name}': Return to base determined possible, but no valid map_pose or gps_navsatfix. Stopping.")
                self.stop_navigation()
        else:
            self.get_logger().warn(f"Cannot navigate to base '{target_name}'. Insufficient data or config. Returning to IDLE.")
            self.stop_navigation()

    def _start_gps_direct_navigation_loop(self):
        """Запускает таймер для цикла прямого GPS управления."""
        if not self.target_gps_for_direct_nav:
            self.get_logger().error("Cannot start GPS direct navigation: target_gps_for_direct_nav is None.")
            self.stop_navigation()
            return

        self.get_logger().info(f"Starting direct GPS navigation control loop. Target: "
                               f"Lat {self.target_gps_for_direct_nav.latitude:.6f}, "
                               f"Lon {self.target_gps_for_direct_nav.longitude:.6f}")
        
        gps_mode_msg = Bool()
        gps_mode_msg.data = True 
        self.gps_mode_control_pub.publish(gps_mode_msg)

        # Публикуем целевые GPS координаты (для отладки или если какой-то узел их использует)
        debug_gps_target_msg = NavSatFix()
        debug_gps_target_msg.latitude = self.target_gps_for_direct_nav.latitude
        debug_gps_target_msg.longitude = self.target_gps_for_direct_nav.longitude
        debug_gps_target_msg.header.stamp = self.get_clock().now().to_msg()
        debug_gps_target_msg.header.frame_id = 'gps_direct_target' 
        self.gps_target_pub.publish(debug_gps_target_msg)

        if self.gps_navigation_timer: # Отменяем старый таймер, если он был
            self.gps_navigation_timer.cancel()
        
        if self.gps_timer_period <= 0:
            self.get_logger().error(f"GPS timer period is invalid ({self.gps_timer_period}s). Cannot start GPS control loop.")
            self.stop_navigation()
            return

        self.gps_navigation_timer = self.create_timer(
            self.gps_timer_period, 
            self.update_gps_navigation_control_loop
        )
        self.get_logger().info(f"GPS control timer started with period {self.gps_timer_period:.2f}s.")


    def update_gps_navigation_control_loop(self):
        # Этот метод вызывается таймером
        if not (self.current_state == 'NAVIGATING_GPS' or self.current_state == 'RETURNING_TO_BASE_GPS'):
            self.get_logger().warn(f"GPS navigation control loop called in incorrect state: '{self.current_state}'. Stopping loop.")
            self._stop_gps_direct_navigation_loop(publish_stop_cmd=False) # Движение уже должно быть остановлено через stop_navigation
            return

        if self.current_gps_location is None or self.target_gps_for_direct_nav is None:
            self.get_logger().warn("GPS control: Missing current robot GPS or target GPS data. Publishing zero velocity.")
            self.cmd_vel_pub.publish(Twist()) 
            return

        # 1. Расчет расстояния и азимута до цели
        R_EARTH = 6371000.0  # Радиус Земли в метрах
        lat1_rad = math.radians(self.current_gps_location.latitude)
        lon1_rad = math.radians(self.current_gps_location.longitude)
        lat2_rad = math.radians(self.target_gps_for_direct_nav.latitude)
        lon2_rad = math.radians(self.target_gps_for_direct_nav.longitude)

        d_lon = lon2_rad - lon1_rad
        d_lat = lat2_rad - lat1_rad

        local_x_to_target = R_EARTH * d_lon * math.cos((lat1_rad + lat2_rad) / 2.0) # Восток
        local_y_to_target = R_EARTH * d_lat # Север
        
        distance_to_target = math.sqrt(local_x_to_target**2 + local_y_to_target**2)
        
        # 2. Проверка достижения цели
        if distance_to_target < self.gps_nav_tolerance:
            self.get_logger().info(f"GPS Target Reached! Distance: {distance_to_target:.2f}m (tolerance: {self.gps_nav_tolerance}m)")
            self._stop_gps_direct_navigation_loop() # Останавливаем таймер и движение
            
            if self.current_state == 'NAVIGATING_GPS':
                self.set_state('AT_DESTINATION')
                self.start_time_at_destination_timer()
            elif self.current_state == 'RETURNING_TO_BASE_GPS':
                self.get_logger().info("Successfully returned to base (GPS navigation). Processing task completion.")
                self._stop_gps_direct_navigation_loop() # Останавливаем цикл GPS перед обработкой завершения
                self._process_task_completion() # <--- НОВЫЙ ВЫЗОВ
                # self.set_state('IDLE') и сброс current_destination_id теперь произойдет внутри _process_task_completion -> stop_navigation
            return

        twist_cmd = Twist()
        obstacle_directly_in_front = False

        # 3. Проверка препятствий по лидару
        if self.current_scan and self.current_scan.ranges:
            num_ranges = len(self.current_scan.ranges)
            # Убедимся, что angle_increment не ноль, чтобы избежать деления на ноль
            if num_ranges > 0 and self.current_scan.angle_increment > 1e-6 :
                center_index_float = (0.0 - self.current_scan.angle_min) / self.current_scan.angle_increment # Угол 0.0 это прямо перед роботом
                center_index = int(round(center_index_float))
                
                view_angle_rad_half = math.radians(15.0 / 2.0) # +/- 7.5 градусов от центрального луча
                num_rays_offset = int(round(view_angle_rad_half / self.current_scan.angle_increment))

                min_idx = max(0, center_index - num_rays_offset)
                max_idx = min(num_ranges - 1, center_index + num_rays_offset)
                
                # self.get_logger().debug(f"Scan check: center_idx={center_index}, min_idx={min_idx}, max_idx={max_idx}, num_ranges={num_ranges}")

                for i in range(min_idx, max_idx + 1):
                    dist = self.current_scan.ranges[i]
                    if not (math.isinf(dist) or math.isnan(dist)) and \
                       self.current_scan.range_min < dist < self.obstacle_detection_distance:
                        obstacle_directly_in_front = True
                        break
            else:
                 self.get_logger().warn("Lidar data has no ranges or angle_increment is zero.")
        # else:
            # self.get_logger().warn("No Lidar data (/scan) available for obstacle check.")


        # Вычисление bearing_to_target_rad и angle_diff_rad для логирования и управления
        bearing_to_target_rad = math.atan2(local_x_to_target, local_y_to_target)  # Угол от Y (Север) к X (Восток)
        current_robot_yaw_rad_for_logic = None  # Изначально неизвестен

        if self.current_true_heading_rad is not None:
            current_robot_yaw_rad_for_logic = self.current_true_heading_rad
        elif self.current_map_pose:
            current_robot_yaw_rad_for_logic = self.quaternion_to_yaw(self.current_map_pose.orientation)
            self.get_logger().warn("Compass data N/A, using AMCL pose yaw for GPS navigation.")
        else:
            current_robot_yaw_rad_for_logic = 0.0  # Крайний случай
            self.get_logger().warn("Compass and AMCL data N/A. Assuming yaw=0 for GPS navigation.")

        # Если yaw всё ещё None (на всякий случай):
        if current_robot_yaw_rad_for_logic is None:
            self.get_logger().error("Critical error: Robot yaw could not be determined. Stopping movement.")
            self.cmd_vel_pub.publish(Twist())  # Останавливаем движение
            return

        # Используем yaw для расчёта угла:
        angle_diff_rad = self.normalize_angle(bearing_to_target_rad - current_robot_yaw_rad_for_logic)

        if obstacle_directly_in_front:
            # self.get_logger().warn("GPS control: Obstacle detected directly in front!") # Будет спамить, используем debug
            self.get_logger().debug("GPS control: Obstacle detected directly in front!")
            twist_cmd.linear.x = 0.0
            # Простое решение: пытаемся немного повернуть (например, всегда вправо)
            twist_cmd.angular.z = -self.turn_speed_gps # Поворот вправо (отрицательный Z)
        else:
            # 4. Логика движения и поворота
            # Поворот к цели
            # Порог для поворота (например, 10 градусов = ~0.17 рад)
            if abs(angle_diff_rad) > math.radians(10.0): 
                twist_cmd.angular.z = self.turn_speed_gps * math.copysign(1.0, angle_diff_rad)
                # Уменьшаем линейную скорость при значительном повороте
                twist_cmd.linear.x = self.forward_speed_gps * 0.25 
            else: # Если курс примерно на цель, двигаемся вперед
                twist_cmd.linear.x = self.forward_speed_gps
                # Небольшая коррекция курса, если нужно, или 0
                twist_cmd.angular.z = self.turn_speed_gps * 0.2 * math.copysign(1.0, angle_diff_rad) if abs(angle_diff_rad) > math.radians(2.0) else 0.0
        
        # ЛОГИРОВАНИЕ ПЕРЕД ПУБЛИКАЦИЕЙ КОМАНД
        self.get_logger().debug(f"GPS Nav Loop: Dist: {distance_to_target:.2f}m, Obstacle: {obstacle_directly_in_front}, "
                               f"BearingTarget: {math.degrees(bearing_to_target_rad):.1f}deg, RobotYaw: {math.degrees(current_robot_yaw_rad_for_logic):.1f}deg, "
                               f"AngleDiff: {math.degrees(angle_diff_rad):.1f}deg, "
                               f"Cmd L.X: {twist_cmd.linear.x:.2f}, Cmd A.Z: {twist_cmd.angular.z:.2f}")
        
        self.cmd_vel_pub.publish(twist_cmd)

    def _stop_gps_direct_navigation_loop(self, publish_stop_cmd=True):
        """Останавливает таймер и движение для прямого GPS управления."""
        if self.gps_navigation_timer is not None:
            self.get_logger().info("Stopping direct GPS navigation control loop timer.")
            self.gps_navigation_timer.cancel()
            self.gps_navigation_timer = None
        
        if publish_stop_cmd:
            self.get_logger().info("Publishing zero velocity to /cmd_vel.")
            self.cmd_vel_pub.publish(Twist())

        self.target_gps_for_direct_nav = None # Сбрасываем цель

        gps_mode_msg = Bool()
        gps_mode_msg.data = False # Выключаем флаг режима GPS
        self.gps_mode_control_pub.publish(gps_mode_msg)

    def stop_navigation(self):
        self.get_logger().info("STOPPING ALL NAVIGATION ACTIVITIES.")
        
        if self.active_nav2_goal_handle is not None:
            self.get_logger().info('Cancelling active Nav2 goal.')
            self.active_nav2_goal_handle.cancel_goal_async()
            self.active_nav2_goal_handle = None 
        
        self._stop_gps_direct_navigation_loop(publish_stop_cmd=True)

        if self.time_at_destination_timer is not None:
            self.time_at_destination_timer.cancel()
            self.time_at_destination_timer = None
            self.get_logger().info("Cancelled 'time at destination' timer.")

        if self.waiting_for_gps_timeout_timer is not None:
            self.waiting_for_gps_timeout_timer.cancel()
            self.waiting_for_gps_timeout_timer = None
            self.get_logger().info("Cancelled 'waiting for dynamic target GPS timeout' timer.")
        
        self.pending_destination_info_for_gps_wait = None
        self.perform_mapping_for_current_task = False 
        self.get_logger().info("Mapping flag 'perform_mapping_for_current_task' reset.")

        self.set_state('IDLE')
        self.current_destination_id = None 
        self.get_logger().info("Navigation fully stopped. State set to IDLE.")

    def _process_task_completion(self):
        self.get_logger().info("Task sequence considered complete (returned to base or task finished).")

        if self.perform_mapping_for_current_task and self.current_task_map_path:
            self.get_logger().info(f"Mapping was active for this task. Attempting to save map associated with: {self.current_task_map_path}")

            # Извлекаем базовое имя и путь без расширения .yaml для сервиса SaveMap
            # self.current_task_map_path у нас содержит полный путь + имя + .yaml
            # Сервис ожидает map_url без расширения.
            map_base_url_for_saving = os.path.splitext(self.current_task_map_path)[0]

            # Вызываем ранее созданный метод для сохранения карты
            # call_save_map_service вернет True или False
            map_saved_successfully = self.call_save_map_service(map_base_url_for_saving)

            if map_saved_successfully:
                self.get_logger().info(f"Map '{map_base_url_for_saving}' seems to have been saved successfully after task completion.")
                # Теперь self.current_task_specific_map_exists для этой карты должно быть True
                # Мы могли бы обновить этот флаг для уже загруженной конфигурации, но
                # при следующем выборе этой цели он все равно будет проверен через os.path.exists()
            else:
                self.get_logger().error(f"Failed to save map '{map_base_url_for_saving}' after task completion.")
        else:
            self.get_logger().info("No active mapping task was flagged for this completed sequence, or map path was not set. Map will not be saved.")

        # После попытки сохранения карты (или если оно не требовалось),
        # полностью останавливаем навигацию и переходим в IDLE.
        # stop_navigation() также сбросит self.perform_mapping_for_current_task для СЛЕДУЮЩЕЙ задачи.
        self.get_logger().info("Proceeding to stop all activities and set state to IDLE.")
        self.stop_navigation()

    def quaternion_to_yaw(self, q_geom_msg_pose_orientation):
        """Конвертирует geometry_msgs/Quaternion в угол рыскания (yaw) вокруг оси Z."""
        q = q_geom_msg_pose_orientation
        # Рыскание (yaw) вокруг оси Z ( Tait-Bryan углы ZYX')
        # siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Формула для кватерниона (x,y,z,w) -> рыскание (Z)
        # t0 = +2.0 * (q.w * q.x + q.y * q.z)
        # t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        # roll_x = math.atan2(t0, t1) # Крен (X)
     
        # t2 = +2.0 * (q.w * q.y - q.z * q.x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # pitch_y = math.asin(t2) # Тангаж (Y)
     
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4) # Рыскание (Z)
        return yaw_z


    def normalize_angle(self, angle_rad):
        """Нормализует угол в диапазон [-PI, PI]."""
        while angle_rad > math.pi:
            angle_rad -= 2.0 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2.0 * math.pi
        return angle_rad

    def destroy_node(self): # Переопределение для корректной остановки
        self.get_logger().info("Destroying NavigationManagerNode...")
        self.stop_navigation() # Убедимся, что все таймеры и задачи остановлены
        # Дополнительная очистка ресурсов, если потребуется (например, закрытие файлов)
        super().destroy_node()

    def call_save_map_service(self, map_base_url_without_extension):
        """
        Вызывает сервис /slam_toolbox/save_map для сохранения карты.
        map_base_url_without_extension: базовый путь и имя файла карты без расширения
                                        (например, "/root/ros2_ws/maps_storage/destmap1")
        Возвращает True в случае успеха, False в случае ошибки.
        """
        if not self.save_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('SaveMap service /slam_toolbox/save_map not available.')
            return False

        request = SaveMap.Request()
        # map_topic обычно "map", если slam_toolbox публикует карту в этот топик
        # request.map_topic = "map" # В ROS2 Humble SaveMap.srv не имеет map_topic, он берется из slam_toolbox
        request.map_url = map_base_url_without_extension 
        request.image_format = "pgm"  # Стандартный формат для Nav2
        request.map_mode = "trinary" # trinary (0, 100, -1), scale, or raw
        request.free_thresh = 0.25   # Стандартные значения из map_saver
        request.occupied_thresh = 0.65

        self.get_logger().info(f"Requesting to save map via service: '{map_base_url_without_extension}.yaml/.pgm'...")
        future = self.save_map_client.call_async(request)

        # Ожидаем завершения вызова сервиса (блокирующий вызов)
        # Это упрощенный вариант для синхронного ожидания.
        # В более сложных сценариях можно использовать add_done_callback.
        try:
            # Увеличим таймаут ожидания, так как сохранение карты может занять время
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0) 
            if future.done():
                response = future.result()
                if response is not None and hasattr(response, 'result') and response.result:
                    self.get_logger().info(f"Map successfully saved to '{map_base_url_without_extension}.yaml/.pgm'")
                    return True
                elif response is not None: # slam_toolbox в Humble возвращает пустой ответ при успехе.
                                        # Отсутствие исключения и не None результат считаем успехом.
                    self.get_logger().info(f"Map save service call completed (likely success) for '{map_base_url_without_extension}'. Check files.")
                    # Проверка фактического существования файла может быть добавлена здесь, если необходимо
                    # Например, time.sleep(0.5) а затем os.path.exists(map_base_url_without_extension + ".yaml")
                    return True # Предполагаем успех, если нет ошибок и результат не явно False
                else: # future.result() вернул None (возможно, таймаут или другая проблема)
                    self.get_logger().error(f"Failed to save map. Service call did not return a valid response or timed out for '{map_base_url_without_extension}'.")
                    return False
            else: # future не завершился за таймаут
                self.get_logger().error(f"SaveMap service call timed out for '{map_base_url_without_extension}'.")
                return False
        except Exception as e:
            self.get_logger().error(f"SaveMap service call failed with exception for '{map_base_url_without_extension}': {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation manager node stopped by user (KeyboardInterrupt)")
    # except Exception as e: # Можно добавить обработку других исключений при spin
    #    node.get_logger().fatal(f"Unhandled exception in spin: {e}")
    finally:
        # Корректное уничтожение узла вызовет destroy_node()
        if rclpy.ok(): # Проверяем, не был ли shutdown вызван где-то еще
            node.destroy_node() 
            rclpy.shutdown()

if __name__ == '__main__':
    main()