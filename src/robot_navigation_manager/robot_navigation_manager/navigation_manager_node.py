#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import NavSatFix, LaserScan # LaserScan добавлен
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist # PoseStamped может понадобиться для Nav2
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import yaml
import os
import math
import time
# import sys # Не используется, можно удалить

class NavigationManagerNode(Node):

    def __init__(self):
        super().__init__('navigation_manager_node')

        # --- Параметры ---
        # Существующие параметры
        # Используем ваш подтвержденный путь к файлу конфигурации
        self.declare_parameter('config_file', '/root/ros2_ws/src/robot_navigation_manager/config/destinations.yaml')
        self.declare_parameter('time_at_destination_duration', 10.0) # Значение по умолчанию
        self.declare_parameter('gps_logging_interval', 5.0) # Интервал логирования GPS/AMCL

        # НОВЫЕ параметры для GPS-навигации и управления
        self.declare_parameter('gps_nav_tolerance_meters', 2.0)  # Допуск для GPS цели в метрах
        self.declare_parameter('gps_obstacle_detection_dist_meters', 0.7) # Дистанция обнаружения препятствий
        self.declare_parameter('gps_forward_speed', 0.2)         # Линейная скорость в GPS режиме (м/с)
        self.declare_parameter('gps_turn_speed', 0.3)            # Угловая скорость в GPS режиме (рад/с)
        self.declare_parameter('gps_control_frequency_hz', 5.0)  # Частота цикла управления GPS (Гц)

        # --- Состояния и переменные ---
        # Существующие
        self.current_state = 'IDLE'
        self.current_destination_id = None
        self.current_gps_location = None   # NavSatFix
        self.dynamic_gps_target = None     # NavSatFix (Для цели 1 из /gps/fix)
        self.current_map_pose = None       # geometry_msgs/Pose (из AMCL)

        self.destinations = {}
        self.base_location = None
        self.map_areas = {}

        self.time_at_destination_timer = None
        # self.time_at_destination_duration # Будет перезаписано из параметра
        self.last_gps_log_time = 0.0       # Для троттлинга логов GPS/AMCL
        # self.gps_logging_interval       # Будет перезаписано из параметра

        self.manual_control_active = False # Пока не используется явно

        # НОВЫЕ переменные для GPS-навигации и Nav2
        self.current_scan = None              # sensor_msgs/LaserScan
        self.target_gps_for_direct_nav = None # NavSatFix: Текущая GPS-цель для прямого управления
        self.gps_navigation_timer = None      # Таймер для цикла прямого GPS управления
        self.active_nav2_goal_handle = None   # Для возможности отмены цели Nav2

        # --- Конфигурация ---
        self.load_configuration() # Загрузка YAML (здесь может быть переопределен time_at_destination_duration из файла)

        # --- ПОЛУЧЕНИЕ ЗНАЧЕНИЙ ВСЕХ ПАРАМЕТРОВ (включая новые) ---
        self.time_at_destination_duration = float(self.get_parameter('time_at_destination_duration').value)
        self.gps_logging_interval = float(self.get_parameter('gps_logging_interval').value)
        
        self.gps_nav_tolerance = self.get_parameter('gps_nav_tolerance_meters').value
        self.obstacle_detection_distance = self.get_parameter('gps_obstacle_detection_dist_meters').value
        self.forward_speed_gps = self.get_parameter('gps_forward_speed').value
        self.turn_speed_gps = self.get_parameter('gps_turn_speed').value
        gps_control_frequency = self.get_parameter('gps_control_frequency_hz').value
        try:
            self.gps_timer_period = 1.0 / gps_control_frequency if gps_control_frequency > 0 else 0.2 # Защита от деления на ноль
        except ZeroDivisionError:
            self.get_logger().warn("gps_control_frequency_hz is 0, defaulting timer period to 0.2s (5Hz)")
            self.gps_timer_period = 0.2


        # <<< БЛОК ЛОГИРОВАНИЯ ПОЛУЧЕННЫХ ПАРАМЕТРОВ >>>
        self.get_logger().info(f"--- Retrieved Parameters ---")
        self.get_logger().info(f"Config file path: {self.get_parameter('config_file').value}") # Логируем и путь к конфигу
        self.get_logger().info(f"Time at destination duration: {self.time_at_destination_duration} s")
        self.get_logger().info(f"GPS logging interval: {self.gps_logging_interval} s")
        self.get_logger().info(f"GPS navigation tolerance: {self.gps_nav_tolerance} m")
        self.get_logger().info(f"GPS obstacle detection distance: {self.obstacle_detection_distance} m")
        self.get_logger().info(f"GPS forward speed: {self.forward_speed_gps} m/s")
        self.get_logger().info(f"GPS turn speed: {self.turn_speed_gps} rad/s")
        self.get_logger().info(f"GPS control frequency: {gps_control_frequency} Hz")
        self.get_logger().info(f"Calculated GPS control timer period: {self.gps_timer_period} s")
        self.get_logger().info(f"----------------------------")
        # <<< КОНЕЦ БЛОКА ЛОГИРОВАНИЯ ПАРАМЕТРОВ >>>

        # --- Подписки ---
        self.destination_sub = self.create_subscription(Int32, '/destination_id', self.destination_callback, 10)
        self.robot_gps_sub = self.create_subscription(NavSatFix, '/robot/gps/fix', self.robot_gps_callback, 10)
        self.dynamic_gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_fix_callback, 10) # Для цели 1
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data) # QoS для сенсоров
        
        self.get_logger().info('Subscribed to /destination_id, /robot/gps/fix, /gps/fix (dynamic target 1), /amcl_pose, /scan')

        # --- Публикации ---
        self.gps_target_pub = self.create_publisher(NavSatFix, '/gps_target', 10) # Может быть полезно для отладки
        self.gps_mode_control_pub = self.create_publisher(Bool, '/gps_mode_control', 10) # Индикация режима
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Created publishers for /gps_target, /gps_mode_control, /cmd_vel')

        # --- Nav2 Action Client ---
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 NavigateToPose Action Client created')
        
        # --- Финальная инициализация ---
        self.get_logger().info('Navigation Manager Node fully initialized.')
        self.set_state('IDLE')

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

    # --- Navigation Logic Methods ---
    def set_state(self, new_state):
        if self.current_state != new_state:
            self.get_logger().info(f"State transition: {self.current_state} -> {new_state}")
            self.current_state = new_state

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
        self.get_logger().info(f"Starting navigation process for '{target_name}'.")
        
        can_navigate, use_map_nav, target_map_pose, target_gps_navsatfix = \
            self._determine_navigation_parameters(destination_info, is_base_return=False)

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
                self.get_logger().info("Successfully returned to base (map navigation). Setting state to IDLE.")
                self.set_state('IDLE')
                self.current_destination_id = None # Сбрасываем цель после возврата на базу
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
                self.get_logger().info("Successfully returned to base (GPS navigation). Setting state to IDLE.")
                self.set_state('IDLE')
                self.current_destination_id = None
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
        bearing_to_target_rad = math.atan2(local_x_to_target, local_y_to_target) # Угол от Y (Север) к X (Восток)
        current_robot_yaw_rad_for_logic = 0.0 
        if self.current_map_pose: # Используем AMCL yaw, если доступен
            current_robot_yaw_rad_for_logic = self.quaternion_to_yaw(self.current_map_pose.orientation)
        
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
        
        # 1. Отмена активной цели Nav2
        if self.active_nav2_goal_handle is not None:
            self.get_logger().info('Attempting to cancel active Nav2 goal.')
            # Не ждем результата отмены, просто отправляем запрос
            self.active_nav2_goal_handle.cancel_goal_async()
            self.active_nav2_goal_handle = None 
        
        # 2. Остановка прямого GPS управления (таймер и движение)
        self._stop_gps_direct_navigation_loop(publish_stop_cmd=True) # Отправит Twist() с нулями

        # 3. Остановка таймера пребывания в пункте назначения
        if self.time_at_destination_timer is not None:
            self.time_at_destination_timer.cancel()
            self.time_at_destination_timer = None
            self.get_logger().info("Cancelled 'time at destination' timer.")

        # 4. Сброс состояния и текущей цели
        self.set_state('IDLE')
        self.current_destination_id = None 
        self.get_logger().info("Navigation fully stopped. State set to IDLE.")

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