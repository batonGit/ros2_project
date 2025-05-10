#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
# Импорт для Nav2 Action Client
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import yaml
import os
import math
import time
import sys

class NavigationManagerNode(Node):

    def __init__(self):
        super().__init__('navigation_manager_node')

        # --- Параметры ---
        self.declare_parameter('config_file', '/root/ros2_ws/src/robot_navigation_manager/config/destinations.yaml')
        self.declare_parameter('time_at_destination_duration', 10.0)
        self.declare_parameter('gps_logging_interval', 5.0)  # New parameter for GPS logging interval

        # --- Состояния и переменные ---
        self.current_state = 'IDLE'
        self.current_destination_id = None
        self.current_gps_location = None
        self.dynamic_gps_target = None
        self.current_map_pose = None

        self.destinations = {}
        self.base_location = None
        self.map_areas = {}

        self.time_at_destination_timer = None
        self.time_at_destination_duration = 10.0
        self.last_gps_log_time = 0.0
        self.gps_logging_interval = 5.0

        self.manual_control_active = False

        # --- Конфигурация ---
        self.load_configuration()

        # --- Подписки ---
        self.destination_sub = self.create_subscription(
            Int32,
            '/destination_id',
            self.destination_callback,
            10)
        self.get_logger().info('Subscribed to /destination_id')

        self.robot_gps_sub = self.create_subscription(
            NavSatFix,
            '/robot/gps/fix',
            self.robot_gps_callback,
            10)
        self.get_logger().info('Subscribed to /robot/gps/fix')

        self.dynamic_gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_fix_callback,
            10)
        self.get_logger().info('Subscribed to /gps/fix for dynamic target')

        self.amcl_pose_sub = self.create_subscription(
             PoseWithCovarianceStamped,
             '/amcl_pose',
             self.amcl_pose_callback,
             10)
        self.get_logger().info('Subscribed to /amcl_pose')

        # --- Публикации ---
        self.gps_target_pub = self.create_publisher(NavSatFix, '/gps_target', 10)
        self.get_logger().info('Publishing to /gps_target')

        self.gps_mode_control_pub = self.create_publisher(Bool, '/gps_mode_control', 10)
        self.get_logger().info('Publishing to /gps_mode_control')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Publishing to /cmd_vel (for stopping)')

        # --- Nav2 Action Client ---
        self._nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose')
        self.get_logger().info('Nav2 NavigateToPose Action Client created')

        # --- Финальная инициализация ---
        self.time_at_destination_duration = float(self.get_parameter('time_at_destination_duration').value)
        self.gps_logging_interval = float(self.get_parameter('gps_logging_interval').value)
        self.get_logger().info(f"Time at destination duration set to: {self.time_at_destination_duration} seconds")
        self.get_logger().info('Navigation Manager Node initialized')
        self.set_state('IDLE')

    def load_configuration(self):
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.get_logger().info(f"Attempting to load configuration from {config_file_path}")

        if not os.path.exists(config_file_path):
            self.get_logger().error(f"Configuration file not found at {config_file_path}. Navigation will be limited.")
            self.destinations = {}
            self.base_location = None
            self.map_areas = {}
            return

        try:
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info("Configuration file loaded successfully.")

            # Загрузка пунктов назначения
            self.destinations = config.get('destinations', {})
            self.get_logger().info(f"Loaded {len(self.destinations)} destinations.")

            # Загрузка Базы
            self.base_location = config.get('base', None)
            if self.base_location:
                self.get_logger().info("Loaded base location.")
            else:
                self.get_logger().warn("No 'base' section found in config.")

            # Загрузка областей карт
            self.map_areas = config.get('map_areas', {})
            self.get_logger().info(f"Loaded {len(self.map_areas)} map areas.")

            # Загрузка времени пребывания в пункте назначения
            if 'time_at_destination_duration' in config:
                duration_from_config = float(config['time_at_destination_duration'])
                self.set_parameters([rclpy.Parameter('time_at_destination_duration', 
                                   rclpy.Parameter.Type.DOUBLE, 
                                   duration_from_config)])
                self.get_logger().info(f"Set time_at_destination_duration parameter to {duration_from_config}.")

        except Exception as e:
            self.get_logger().error(f"Failed to parse configuration file: {e}. Navigation will be limited.")
            self.destinations = {}
            self.base_location = None
            self.map_areas = {}

    # --- Callback Methods ---
    def robot_gps_callback(self, msg):
        self.current_gps_location = msg
        current_time = time.time()
        if current_time - self.last_gps_log_time > self.gps_logging_interval:
            self.get_logger().info(f"Robot GPS: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}", 
                                  throttle_duration_sec=self.gps_logging_interval)
            self.last_gps_log_time = current_time

    def gps_fix_callback(self, msg):
        dest1_info = self.destinations.get('1', {})
        if dest1_info.get('source') == 'topic' and dest1_info.get('topic_name') == '/gps/fix':
            self.dynamic_gps_target = msg
            current_time = time.time()
            if current_time - self.last_gps_log_time > self.gps_logging_interval:
                self.get_logger().info(f"Dynamic GPS target: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}",
                                      throttle_duration_sec=self.gps_logging_interval)
                self.last_gps_log_time = current_time

    def amcl_pose_callback(self, msg):
        self.current_map_pose = msg.pose.pose
        current_time = time.time()
        if current_time - self.last_gps_log_time > self.gps_logging_interval:
            self.get_logger().info(f"AMCL Pose: x {msg.pose.pose.position.x:.2f}, y {msg.pose.pose.position.y:.2f}",
                                  throttle_duration_sec=self.gps_logging_interval)
            self.last_gps_log_time = current_time

    def destination_callback(self, msg):
        destination_id = msg.data
        self.get_logger().info(f"Received destination ID: {destination_id}")

        if destination_id == 0:
            self.get_logger().info("Received ID 0: Stopping navigation.")
            self.stop_navigation()
            self.set_state('IDLE')
            return

        dest_key = str(destination_id)
        if dest_key not in self.destinations:
            self.get_logger().warn(f"Destination ID {destination_id} not found in configuration.")
            self.set_state('IDLE')
            return

        destination_info = self.destinations[dest_key]
        self.current_destination_id = destination_id
        self.set_state('PLANNING_NAVIGATION')
        self.start_navigation_to_destination(destination_info)

    # --- Navigation Logic Methods ---
    def set_state(self, state):
        if self.current_state != state:
            self.get_logger().info(f"State transition: {self.current_state} -> {state}")
            self.current_state = state

    def start_navigation_to_destination(self, destination_info):
        target_gps = None
        target_pose = None
        use_map_navigation = False
        can_navigate = False

        source_type = destination_info.get('source', 'recorded')

        if source_type == 'topic':
            topic_name = destination_info.get('topic_name')
            if topic_name == '/gps/fix' and self.dynamic_gps_target is not None:
                target_gps = self.dynamic_gps_target
                self.get_logger().info(f"Using dynamic GPS target from {topic_name}")
                can_navigate = True
                use_map_navigation = False
            else:
                self.get_logger().warn(f"Destination source is 'topic' but topic_name '{topic_name}' not supported")
                can_navigate = False

        elif source_type == 'recorded':
            if ('map_target_pose' in destination_info and 
                'map_name' in destination_info and 
                self.current_gps_location and 
                self.current_map_pose):
                
                map_name = destination_info['map_name']
                if map_name in self.map_areas and 'bounds' in self.map_areas[map_name]:
                    bounds = self.map_areas[map_name]['bounds']
                    if (self.current_gps_location.latitude >= bounds['min_lat'] and
                        self.current_gps_location.latitude <= bounds['max_lat'] and
                        self.current_gps_location.longitude >= bounds['min_lon'] and
                        self.current_gps_location.longitude <= bounds['max_lon']):
                        
                        target_pose = destination_info['map_target_pose']
                        use_map_navigation = True
                        can_navigate = True
                        self.get_logger().info(f"Using map navigation in area '{map_name}'")
                    else:
                        self.get_logger().info(f"Outside map area '{map_name}'. Using GPS navigation.")
                else:
                    self.get_logger().warn(f"Map area '{map_name}' not found or lacks bounds")

            if not use_map_navigation and 'gps_coords' in destination_info:
                target_gps = destination_info['gps_coords']
                can_navigate = True
                self.get_logger().info("Using recorded GPS coordinates")
            elif not use_map_navigation:
                self.get_logger().error("Destination lacks valid map_target_pose or gps_coords")
                can_navigate = False

        else:
            self.get_logger().error(f"Unknown source type '{source_type}'")
            can_navigate = False

        if can_navigate:
            gps_mode_msg = Bool()
            gps_mode_msg.data = False
            self.gps_mode_control_pub.publish(gps_mode_msg)

            if use_map_navigation:
                self.set_state('NAVIGATING_MAP')
                self.send_nav2_goal(target_pose)
            elif target_gps is not None:
                self.set_state('NAVIGATING_GPS')
                gps_target_msg = NavSatFix()
                
                if isinstance(target_gps, NavSatFix):
                    gps_target_msg.latitude = target_gps.latitude
                    gps_target_msg.longitude = target_gps.longitude
                elif isinstance(target_gps, dict) and 'latitude' in target_gps and 'longitude' in target_gps:
                    gps_target_msg.latitude = float(target_gps['latitude'])
                    gps_target_msg.longitude = float(target_gps['longitude'])
                else:
                    self.get_logger().error("Invalid GPS target format")
                    self.stop_navigation()
                    return

                gps_target_msg.header.stamp = self.get_clock().now().to_msg()
                gps_target_msg.header.frame_id = 'gps'

                self.gps_target_pub.publish(gps_target_msg)
                self.get_logger().info(f"Published GPS target: Lat {gps_target_msg.latitude:.6f}, Lon {gps_target_msg.longitude:.6f}")

                gps_mode_msg.data = True
                self.gps_mode_control_pub.publish(gps_mode_msg)
                self.get_logger().info("Activated GPS navigation")
            else:
                self.get_logger().error("Navigation logic failed")
                self.stop_navigation()
        else:
            self.get_logger().warn("Cannot navigate to destination")
            self.set_state('IDLE')

    def send_nav2_goal(self, pose):
        if pose is None:
            self.get_logger().error("Cannot send Nav2 goal - pose is None")
            self.set_state('IDLE')
            return

        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.set_state('IDLE')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal to Nav2')
        self._send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected')
            self.set_state('IDLE')
            return

        self.get_logger().info('Nav2 goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.cancelled():
            self.get_logger().warn('Nav2 goal was cancelled')
            self.set_state('IDLE')
            return
        elif future.exception():
            self.get_logger().error(f'Nav2 goal exception: {future.exception()}')
            self.set_state('IDLE')
            return

        from action_msgs.msg import GoalStatus
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 navigation succeeded")
            self.set_state('AT_DESTINATION')
            self.start_time_at_destination_timer()
        else:
            self.get_logger().error(f"Nav2 navigation failed with status: {status}")
            self.set_state('IDLE')

    def start_time_at_destination_timer(self):
        self.get_logger().info(f"Starting timer for {self.time_at_destination_duration} seconds at destination")
        if self.time_at_destination_timer:
            self.time_at_destination_timer.cancel()

        self.time_at_destination_timer = self.create_timer(
            self.time_at_destination_duration,
            self.time_at_destination_timer_callback)

    def time_at_destination_timer_callback(self):
        self.get_logger().info("Time at destination expired. Returning to base")
        if self.time_at_destination_timer:
            self.time_at_destination_timer.cancel()
            self.time_at_destination_timer = None
        self.return_to_base()

    def return_to_base(self):
        self.get_logger().info("Initiating return to base")
        if not self.base_location:
            self.get_logger().error("Base location not configured")
            self.set_state('IDLE')
            return

        target_gps = None
        target_pose = None
        use_map_for_base = False
        can_navigate_to_base = False

        source_type = self.base_location.get('source', 'recorded')

        if source_type == 'recorded':
            if ('map_target_pose' in self.base_location and 
                'map_name' in self.base_location and 
                self.current_gps_location and 
                self.current_map_pose):
                
                map_name = self.base_location['map_name']
                if map_name in self.map_areas and 'bounds' in self.map_areas[map_name]:
                    bounds = self.map_areas[map_name]['bounds']
                    if (self.current_gps_location.latitude >= bounds['min_lat'] and
                        self.current_gps_location.latitude <= bounds['max_lat'] and
                        self.current_gps_location.longitude >= bounds['min_lon'] and
                        self.current_gps_location.longitude <= bounds['max_lon']):
                        
                        target_pose = self.base_location['map_target_pose']
                        use_map_for_base = True
                        can_navigate_to_base = True
                        self.get_logger().info(f"Using map navigation to base in area '{map_name}'")
                    else:
                        self.get_logger().info("Outside base map area. Using GPS navigation")
                else:
                    self.get_logger().warn("Base map area not found or lacks bounds")

            if not use_map_for_base and 'gps_coords' in self.base_location:
                target_gps = self.base_location['gps_coords']
                can_navigate_to_base = True
                self.get_logger().info("Using recorded GPS coordinates for base")
            elif not use_map_for_base:
                self.get_logger().error("Base lacks valid map_target_pose or gps_coords")
                can_navigate_to_base = False

        else:
            self.get_logger().error(f"Unknown source type '{source_type}' for base")
            can_navigate_to_base = False

        if can_navigate_to_base:
            gps_mode_msg = Bool()
            gps_mode_msg.data = False
            self.gps_mode_control_pub.publish(gps_mode_msg)

            if use_map_for_base:
                self.set_state('RETURNING_TO_BASE_MAP')
                self.send_nav2_goal(target_pose)
            elif target_gps is not None:
                self.set_state('RETURNING_TO_BASE_GPS')
                gps_target_msg = NavSatFix()
                
                if isinstance(target_gps, NavSatFix):
                    gps_target_msg.latitude = target_gps.latitude
                    gps_target_msg.longitude = target_gps.longitude
                elif isinstance(target_gps, dict) and 'latitude' in target_gps and 'longitude' in target_gps:
                    gps_target_msg.latitude = float(target_gps['latitude'])
                    gps_target_msg.longitude = float(target_gps['longitude'])
                else:
                    self.get_logger().error("Invalid GPS target format for base")
                    self.stop_navigation()
                    return

                gps_target_msg.header.stamp = self.get_clock().now().to_msg()
                gps_target_msg.header.frame_id = 'gps'

                self.gps_target_pub.publish(gps_target_msg)
                self.get_logger().info(f"Published GPS target for base: Lat {gps_target_msg.latitude:.6f}, Lon {gps_target_msg.longitude:.6f}")

                gps_mode_msg.data = True
                self.gps_mode_control_pub.publish(gps_mode_msg)
                self.get_logger().info("Activated GPS navigation to base")
            else:
                self.get_logger().error("Return to base logic failed")
                self.stop_navigation()
        else:
            self.get_logger().warn("Cannot navigate to base")
            self.set_state('IDLE')

    def stop_navigation(self):
        self.get_logger().info("Stopping all navigation activities")
        
        gps_mode_msg = Bool()
        gps_mode_msg.data = False
        self.gps_mode_control_pub.publish(gps_mode_msg)

        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().info("Published zero Twist command")

        if self.time_at_destination_timer:
            self.time_at_destination_timer.cancel()
            self.time_at_destination_timer = None
            self.get_logger().info("Cancelled time at destination timer")

        self.set_state('IDLE')
        self.current_destination_id = None

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation manager node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()