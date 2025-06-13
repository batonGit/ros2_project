#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import smbus2
from smbus2 import i2c_msg 
import struct 
import time
import math
import sys 
import os 
import tf_transformations 
from tf2_ros import TransformBroadcaster

class HoverboardControlNode(Node):
    def __init__(self):
        super().__init__('hoverboard_control_node')
        
        self.i2c_bus_number = 1
        self.i2c_address = 0x08
        self.i2c_bus = None 
        
        try:
            self.i2c_bus = smbus2.SMBus(self.i2c_bus_number)
            self.get_logger().info(f"I2C bus {self.i2c_bus_number} opened successfully for Arduino at address {hex(self.i2c_address)}.")
        except FileNotFoundError:
            self.get_logger().error(f"I2C bus {self.i2c_bus_number} not found. Ensure I2C is enabled. Node will not function.")
            return 
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus {self.i2c_bus_number}: {e}. Node will not function.")
            return

        # --- Параметры ---
        self.declare_parameter('compass_rpm_read_period_sec', 0.2)
        self.declare_parameter('motor_command_send_interval_sec', 0.1) 
        self.declare_parameter('i2c_quiet_period_after_motor_cmd_sec', 0.05) 
        self.declare_parameter('stuck_linear_velocity_threshold', 0.05) 
        self.declare_parameter('stuck_angular_velocity_threshold', 0.1) 
        self.declare_parameter('stuck_rpm_threshold', 10.0)           
        self.declare_parameter('stuck_duration_threshold_sec', 1.5)   
        self.declare_parameter('boost_raw_speed_value', 75)        
        self.declare_parameter('boost_steer_value_straight', 0) 
        self.declare_parameter('boost_steer_value_turn_max', 90) 
        self.declare_parameter('boost_duration_sec', 0.75)         
        self.declare_parameter('boost_cooldown_sec', 5.0)
        self.declare_parameter('wheel_radius_meters', 0.12)  # ВАШ РАДИУС
        self.declare_parameter('wheel_base_meters', 0.45)    # ВАША БАЗА
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_link_frame_id', 'base_link')

        # Получаем значения параметров (сокращено для примера, у вас полный список)
        self.compass_read_timer_period = self.get_parameter('compass_rpm_read_period_sec').value
        self.min_send_interval = self.get_parameter('motor_command_send_interval_sec').value
        self.compass_read_quiet_period_after_motor_cmd = self.get_parameter('i2c_quiet_period_after_motor_cmd_sec').value
        self.stuck_lin_vel_thresh = self.get_parameter('stuck_linear_velocity_threshold').value
        self.stuck_ang_vel_thresh = self.get_parameter('stuck_angular_velocity_threshold').value
        self.stuck_rpm_thresh = self.get_parameter('stuck_rpm_threshold').value
        self.stuck_duration_thresh = self.get_parameter('stuck_duration_threshold_sec').value
        self.boost_raw_speed = self.get_parameter('boost_raw_speed_value').value
        self.boost_steer_straight = self.get_parameter('boost_steer_value_straight').value
        self.boost_steer_turn_max = self.get_parameter('boost_steer_value_turn_max').value
        self.boost_duration = self.get_parameter('boost_duration_sec').value
        self.boost_cooldown = self.get_parameter('boost_cooldown_sec').value
        self.wheel_radius = self.get_parameter('wheel_radius_meters').value
        self.wheel_base = self.get_parameter('wheel_base_meters').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_link_frame_id').value

        # Логирование параметров (пример)
        self.get_logger().info(f"Odometry params: WheelRadius={self.wheel_radius}m, WheelBase={self.wheel_base}m")

        cmd_vel_subscriber_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, depth=10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, cmd_vel_subscriber_qos)
        
        self.heading_publisher = self.create_publisher(Float32, '/robot/heading', 10)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_frame, 10) 
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Publishers and TF Broadcaster initialized.")

        self.compass_rpm_odom_timer = self.create_timer(
            self.compass_read_timer_period,
            self.read_compass_rpm_and_publish_odom_callback)
        self.get_logger().info(f"Timer for Odom/Compass/RPM started (period: {self.compass_read_timer_period}s).")

        # Переменные состояния
        self.last_motor_cmd_sent_time_for_quiet_period = 0.0
        self.last_i2c_motor_send_time_for_throttle = 0
        self.latest_l_rpm, self.latest_r_rpm = 0, 0
        self.last_commanded_linear_x, self.last_commanded_angular_z = 0.0, 0.0
        self.stuck_timer_start_time = None
        self.is_boosting = False
        self.boost_end_time = 0.0
        self.last_boost_finish_time = 0.0
        self.current_boost_steer_value = 0
        
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_theta = 0.0 
        self.last_odom_update_time = self.get_clock().now() 
        
        self.get_logger().info('Hoverboard control node initialized')

    def _send_motor_i2c_command(self, steer_command, speed_command) -> bool:
        # ... (этот метод остается без изменений) ...
        if not self.i2c_bus:
            self.get_logger().warn("_send_motor_i2c_command: I2C bus not available.", throttle_duration_sec=5.0)
            return False
        command_packed_bytes = struct.pack('>hh', int(steer_command), int(speed_command)) 
        i2c_payload = [0x01, len(command_packed_bytes)] + list(command_packed_bytes)
        try:
            write_transaction = i2c_msg.write(self.i2c_address, i2c_payload)
            self.i2c_bus.i2c_rdwr(write_transaction)
            current_time = time.time()
            self.last_motor_cmd_sent_time_for_quiet_period = current_time 
            self.last_i2c_motor_send_time_for_throttle = current_time 
            self.get_logger().debug(f'Sent to Arduino (I2C): steer={steer_command}, speed={speed_command}')
            return True
        except OSError as e: 
            err_msg = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
            self.get_logger().error(f'_send_motor_i2c_command: I2C write error: {err_msg}')
            self.get_logger().error(f'_send_motor_i2c_command: Failed to send: steer={steer_command}, speed={speed_command}')
        except Exception as e: 
            self.get_logger().error(f'_send_motor_i2c_command: Generic error: {type(e).__name__} - {e}')
            self.get_logger().error(f'_send_motor_i2c_command: Context: steer={steer_command}, speed={speed_command}')
        return False

    def cmd_vel_callback(self, msg: Twist):
        # ... (этот метод остается без изменений, с инверсией angular_z и логикой буста) ...
        self.get_logger().debug(f"cmd_vel_callback: L.X={msg.linear.x:.2f} A.Z={msg.angular.z:.2f}, Boosting={self.is_boosting}")
        self.last_commanded_linear_x = msg.linear.x
        self.last_commanded_angular_z = msg.angular.z
        current_time = time.time()
        if not self.is_boosting and (current_time - self.last_i2c_motor_send_time_for_throttle < self.min_send_interval):
            self.get_logger().debug(f"Motor cmd throttled. Time since last send: {current_time - self.last_i2c_motor_send_time_for_throttle:.3f}s")
            return
        steer_to_send, speed_to_send = 0, 0
        if self.is_boosting:
            steer_to_send = self.current_boost_steer_value 
            speed_to_send = self.boost_raw_speed 
            self.get_logger().info(f"BOOSTING CMD: Steer={steer_to_send}, RawSpeed={speed_to_send}")
        else:
            angular_z_processed = msg.angular.z * -1.0 
            speed_to_send = int(msg.linear.x * 250)
            steer_to_send = int(max(-90, min(90, angular_z_processed * 90))) 
            speed_to_send = max(-125, min(125, speed_to_send))
        self._send_motor_i2c_command(steer_to_send, speed_to_send)

    def read_compass_rpm_and_publish_odom_callback(self):
        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available for compass/rpm/odom.", throttle_duration_sec=5.0)
            return

        current_callback_time_ros = self.get_clock().now()
        current_callback_time_py = time.time()

        if not self.is_boosting and (current_callback_time_py - self.last_motor_cmd_sent_time_for_quiet_period < self.compass_read_quiet_period_after_motor_cmd):
            self.get_logger().debug(f"Skipping compass/rpm/odom: motor command sent recently "
                                    f"({current_callback_time_py - self.last_motor_cmd_sent_time_for_quiet_period:.3f}s ago).")
            return

        l_rpm_from_arduino = 0 
        r_rpm_from_arduino = 0
        # heading_rad_from_arduino = self.current_odom_theta # Неправильно использовать старую одометрию как курс по умолчанию здесь
        # Лучше использовать None или предыдущее значение с компаса, если оно есть
        # current_heading_degrees_for_log = None # Для логирования, если чтение не удалось

        try:
            bytes_to_read = 6 
            read_transaction = i2c_msg.read(self.i2c_address, bytes_to_read)
            self.i2c_bus.i2c_rdwr(read_transaction)
            data_from_i2c = bytes(read_transaction) 

            if len(data_from_i2c) == bytes_to_read:
                heading_word_times_10, l_rpm_raw, r_rpm_raw = struct.unpack('>hhh', data_from_i2c)
                current_heading_degrees_for_log = float(heading_word_times_10) / 10.0 # Для лога
                heading_rad_from_arduino = math.radians(current_heading_degrees_for_log)
                
                l_rpm_from_arduino = l_rpm_raw
                r_rpm_from_arduino = -r_rpm_raw 
                
                self.latest_l_rpm = l_rpm_from_arduino 
                self.latest_r_rpm = r_rpm_from_arduino
                
                self.get_logger().info(f"Arduino Data: H={current_heading_degrees_for_log:.1f}, L_RPM={self.latest_l_rpm}, R_RPM(corr)={self.latest_r_rpm}")
                heading_msg = Float32(); heading_msg.data = heading_rad_from_arduino
                self.heading_publisher.publish(heading_msg)
            else: 
                self.get_logger().error(f"I2C read (data): Expected {bytes_to_read}, got {len(data_from_i2c)}", throttle_duration_sec=5.0)
                # Если чтение не удалось, не обновляем RPM и heading_rad_from_arduino, они останутся 0 или предыдущими
                # Это приведет к нулевому изменению одометрии в этом цикле, что корректно
        
        except OSError as e: 
             error_message = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
             log_func = self.get_logger().warn if hasattr(e, 'errno') and e.errno == 121 else self.get_logger().error
             log_func(f"I2C read (data) OSError: {error_message}", throttle_duration_sec=2.0 if hasattr(e, 'errno') and e.errno == 121 else 5.0)
        except struct.error as e: 
            self.get_logger().error(f"Unpack I2C data (struct.error): {e}. Rcvd {len(data_from_i2c)}B: {data_from_i2c.hex() if isinstance(data_from_i2c, bytes) else data_from_i2c}")
        except Exception as e: 
            self.get_logger().error(f"Generic I2C read (data): {type(e).__name__} - {e}", throttle_duration_sec=5.0)

        # --- БЛОК ВЫЧИСЛЕНИЯ И ПУБЛИКАЦИИ ОДОМЕТРИИ ---
        dt = 0.0
        # self.last_odom_update_time инициализируется в __init__
        dt_nanosec = current_callback_time_ros.nanoseconds - self.last_odom_update_time.nanoseconds
        dt = dt_nanosec / 1e9 # Конвертируем наносекунды в секунды
        
        self.last_odom_update_time = current_callback_time_ros # Обновляем время для следующего шага

        if dt <= 1e-6: # Пропускаем, если dt слишком мал или это первый вызов после инициализации None
            self.get_logger().debug(f"Odometry dt is {dt:.4f}, too small or negative. Skipping odom calculation.")
        else:
            self.get_logger().debug(f"Calculating Odom: dt={dt:.4f}s, L_RPM={l_rpm_from_arduino}, R_RPM={r_rpm_from_arduino}")
            rpm_to_rad_per_sec = (2 * math.pi) / 60.0
            wheel_l_rad_per_sec = l_rpm_from_arduino * rpm_to_rad_per_sec
            wheel_r_rad_per_sec = r_rpm_from_arduino * rpm_to_rad_per_sec

            v_l_mps = wheel_l_rad_per_sec * self.wheel_radius
            v_r_mps = wheel_r_rad_per_sec * self.wheel_radius

            vx = (v_r_mps + v_l_mps) / 2.0
            wz = 0.0
            if self.wheel_base > 1e-3: # Защита от деления на ноль, если wheel_base не установлен
                 wz = (v_r_mps - v_l_mps) / self.wheel_base
            else:
                 self.get_logger().warn("Wheel base is very small or zero, cannot calculate angular velocity for odometry.", throttle_duration_sec=5.0)


            # Улучшенный метод интегрирования для theta, чтобы уменьшить ошибку при поворотах
            # delta_s = vx * dt
            # delta_theta = wz * dt
            # self.current_odom_x += delta_s * math.cos(self.current_odom_theta + delta_theta / 2.0)
            # self.current_odom_y += delta_s * math.sin(self.current_odom_theta + delta_theta / 2.0)
            # self.current_odom_theta = self.normalize_angle(self.current_odom_theta + delta_theta)

            # Простой метод Эйлера
            delta_x = vx * math.cos(self.current_odom_theta) * dt
            delta_y = vx * math.sin(self.current_odom_theta) * dt
            delta_theta = wz * dt

            self.current_odom_x += delta_x
            self.current_odom_y += delta_y
            self.current_odom_theta = self.normalize_angle(self.current_odom_theta + delta_theta)
            
            self.get_logger().debug(f"Odom Deltas: dx={delta_x:.4f}, dy={delta_y:.4f}, dth={math.degrees(delta_theta):.2f}deg")
            self.get_logger().debug(f"Odom Pose:   X={self.current_odom_x:.2f}, Y={self.current_odom_y:.2f}, Th={math.degrees(self.current_odom_theta):.1f}deg")


            odom_msg = Odometry()
            odom_msg.header.stamp = current_callback_time_ros.to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame

            odom_msg.pose.pose.position.x = self.current_odom_x
            odom_msg.pose.pose.position.y = self.current_odom_y
            odom_msg.pose.pose.position.z = 0.0 

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.current_odom_theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0 
            odom_msg.twist.twist.angular.z = wz

            # --- ИЗМЕНЕННЫЕ КОВАРИАЦИИ ---
            # Меньшие значения для Z, roll, pitch, чтобы не было гигантской плоскости в RViz
            # Для pose: x, y, z, roll, pitch, yaw (главная диагональ)
            odom_msg.pose.covariance[0] = 0.01  # x variance - увеличена неопределенность
            odom_msg.pose.covariance[7] = 0.01  # y variance - увеличена неопределенность
            odom_msg.pose.covariance[14] = 1e-6 # z variance (очень маленькая, т.к. 2D)
            odom_msg.pose.covariance[21] = 1e-6 # roll variance 
            odom_msg.pose.covariance[28] = 1e-6 # pitch variance
            odom_msg.pose.covariance[35] = 0.05 # yaw variance (получше, если есть компас, но это одометрия)
            # Для twist: vx, vy, vz, wx, wy, wz
            odom_msg.twist.covariance[0] = 0.01 # vx variance - увеличена неопределенность
            odom_msg.twist.covariance[7] = 1e-6  # vy
            odom_msg.twist.covariance[14] = 1e-6 # vz
            odom_msg.twist.covariance[21] = 1e-6 # wx
            odom_msg.twist.covariance[28] = 1e-6 # wy
            odom_msg.twist.covariance[35] = 0.02 # wz variance - увеличена неопределенность
            
            self.odom_publisher.publish(odom_msg)
            self.get_logger().debug(f"Published Odom: X={self.current_odom_x:.2f}, Y={self.current_odom_y:.2f}, Th={math.degrees(self.current_odom_theta):.1f}deg. "
                                   f"VX={vx:.2f}, WZ={wz:.2f}")

            t = TransformStamped()
            t.header.stamp = current_callback_time_ros.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.current_odom_x
            t.transform.translation.y = self.current_odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)
        # --- КОНЕЦ БЛОКА ОДОМЕТРИИ ---

        # Логика буста (остается без изменений)
        if self.is_boosting: 
            if current_callback_time_py >= self.boost_end_time:
                self.get_logger().info("Boost sequence finished.")
                self.is_boosting = False
                self.last_boost_finish_time = current_callback_time_py 
                self.get_logger().info(f"Sending post-boost command based on last_cmd: L.X={self.last_commanded_linear_x:.2f}, A.Z={self.last_commanded_angular_z:.2f}")
                angular_z_processed = self.last_commanded_angular_z * -1.0 
                post_boost_speed = int(self.last_commanded_linear_x * 250)
                post_boost_steer = int(max(-90, min(90, angular_z_processed * 90)))
                post_boost_speed = max(-125, min(125, post_boost_speed))
                self._send_motor_i2c_command(post_boost_steer, post_boost_speed)
            return 

        if current_callback_time_py - self.last_boost_finish_time < self.boost_cooldown:
            self.get_logger().debug(f"In boost cooldown. Remaining: {self.boost_cooldown - (current_callback_time_py - self.last_boost_finish_time):.1f}s", throttle_duration_sec=1.0)
            return 

        is_commanded_linear_significant = abs(self.last_commanded_linear_x) >= self.stuck_lin_vel_thresh
        is_commanded_angular_significant = abs(self.last_commanded_angular_z) >= self.stuck_ang_vel_thresh
        should_be_moving_significantly = is_commanded_linear_significant or is_commanded_angular_significant
        
        current_l_rpm_for_stuck = self.latest_l_rpm 
        current_r_rpm_for_stuck = self.latest_r_rpm 
        
        is_physically_stuck = False
        if should_be_moving_significantly:
            if (abs(current_l_rpm_for_stuck) < self.stuck_rpm_thresh) or \
               (abs(current_r_rpm_for_stuck) < self.stuck_rpm_thresh):
                is_physically_stuck = True
                self.get_logger().debug(f"Stuck Condition Met: Commanded move "
                                       f"(L:{self.last_commanded_linear_x:.2f} A:{self.last_commanded_angular_z:.2f}), "
                                       f"and L_RPM({current_l_rpm_for_stuck}) or R_RPM({current_r_rpm_for_stuck}) is below Thresh({self.stuck_rpm_thresh}).")
        
        if is_physically_stuck: 
            if self.stuck_timer_start_time is None: 
                self.stuck_timer_start_time = current_callback_time_py
                self.get_logger().info(f"Potential stuck: Cmd L={self.last_commanded_linear_x:.2f}, A={self.last_commanded_angular_z:.2f}. RPM L={current_l_rpm_for_stuck}, R={current_r_rpm_for_stuck}. Monitoring...")
            elif current_callback_time_py - self.stuck_timer_start_time >= self.stuck_duration_thresh:
                self.get_logger().warn(f"STUCK CONFIRMED: Cmd L={self.last_commanded_linear_x:.2f}, A={self.last_commanded_angular_z:.2f}. "
                                       f"RPM L={current_l_rpm_for_stuck}, R={current_r_rpm_for_stuck} for {current_callback_time_py - self.stuck_timer_start_time:.1f}s. BOOSTING!")
                self.is_boosting = True
                self.boost_end_time = current_callback_time_py + self.boost_duration 
                self.current_boost_steer_value = self.boost_steer_straight 
                if is_commanded_angular_significant and not is_commanded_linear_significant: 
                    processed_angular_for_boost_steer = self.last_commanded_angular_z * -1.0 
                    self.current_boost_steer_value = self.boost_steer_turn_max * math.copysign(1.0, processed_angular_for_boost_steer)
                    self.get_logger().info(f"Boost for rotation: Original AngularZ={self.last_commanded_angular_z:.2f}, Resulting Boost Steer Cmd = {self.current_boost_steer_value}")
                self.get_logger().info(f"Boost activated for {self.boost_duration}s. Steer: {self.current_boost_steer_value}, Speed: {self.boost_raw_speed}")
                self._send_motor_i2c_command(self.current_boost_steer_value, self.boost_raw_speed)
                self.stuck_timer_start_time = None 
        else: 
            if self.stuck_timer_start_time is not None:
                self.get_logger().info("Stuck condition resolved or command to move stopped.")
            self.stuck_timer_start_time = None 

    def normalize_angle(self, angle_rad):
        while angle_rad > math.pi: angle_rad -= 2.0 * math.pi
        while angle_rad < -math.pi: angle_rad += 2.0 * math.pi
        return angle_rad

    def destroy_node(self):
        self.get_logger().info("Shutting down Hoverboard Control Node...")
        if hasattr(self, 'compass_rpm_odom_timer') and self.compass_rpm_odom_timer:
            self.compass_rpm_odom_timer.cancel()
            self.get_logger().info("Compass/RPM/Odom timer cancelled.")
        if self.i2c_bus:
            try:
                self.get_logger().info("Sending stop command to motors before exiting.")
                self._send_motor_i2c_command(0, 0) 
            except Exception as e: 
                self.get_logger().error(f"Exception during sending stop command on destroy: {e}")
            try:
                self.i2c_bus.close()
                self.get_logger().info("I2C bus closed.")
            except Exception as e:
                self.get_logger().error(f"Error closing I2C bus: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    node_creation_successful = False 
    try:
        node = HoverboardControlNode()
        if hasattr(node, 'i2c_bus') and node.i2c_bus: 
            node_creation_successful = True
            rclpy.spin(node)
        else:
            if node is not None: 
                 node.get_logger().fatal("I2C bus was not initialized in __init__. Node did not spin.")
            else: 
                 print("HoverboardControlNode could not be fully created.", file=sys.stderr)
            if rclpy.ok(): rclpy.shutdown()
            return 
    except KeyboardInterrupt:
        if node_creation_successful and node: 
            node.get_logger().info('Node stopped by user (KeyboardInterrupt).')
    except Exception as e:
        log_target = node.get_logger() if node_creation_successful and node else None
        if log_target: log_target.fatal(f"Unhandled exception: {e}", exc_info=True) 
        else: 
            print(f"Unhandled critical exception: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    finally:
        if node and rclpy.ok(): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()