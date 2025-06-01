#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
# --- ДОБАВЛЕНЫ ИМПОРТЫ QoS (если их не было в самом верху) ---
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# --- КОНЕЦ ДОБАВЛЕНИЯ ---
import smbus2
from smbus2 import i2c_msg 
import struct 
import time
import math
import sys 
import os 

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
            self.get_logger().error(f"I2C bus {self.i2c_bus_number} not found. Ensure I2C is enabled on Raspberry Pi. Node will not function correctly.")
            return 
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus {self.i2c_bus_number}: {e}. Node will not function correctly.")
            return

        # --- Параметры ---
        self.declare_parameter('compass_rpm_read_period_sec', 1.0) 
        self.declare_parameter('motor_command_send_interval_sec', 0.1)
        self.declare_parameter('i2c_quiet_period_after_motor_cmd_sec', 0.05) 
        self.declare_parameter('stuck_linear_velocity_threshold', 0.05)
        self.declare_parameter('stuck_rpm_threshold', 10.0)          
        self.declare_parameter('stuck_duration_threshold_sec', 1.5)   
        self.declare_parameter('boost_raw_speed_value', 75)        
        self.declare_parameter('boost_steer_value', 0)             
        self.declare_parameter('boost_duration_sec', 0.75)         
        self.declare_parameter('boost_cooldown_sec', 5.0)          

        # Получаем значения параметров
        self.compass_read_timer_period = self.get_parameter('compass_rpm_read_period_sec').value
        self.min_send_interval = self.get_parameter('motor_command_send_interval_sec').value
        self.compass_read_quiet_period_after_motor_cmd = self.get_parameter('i2c_quiet_period_after_motor_cmd_sec').value
        self.stuck_lin_vel_thresh = self.get_parameter('stuck_linear_velocity_threshold').value
        self.stuck_rpm_thresh = self.get_parameter('stuck_rpm_threshold').value
        self.stuck_duration_thresh = self.get_parameter('stuck_duration_threshold_sec').value
        self.boost_raw_speed = self.get_parameter('boost_raw_speed_value').value
        self.boost_steer = self.get_parameter('boost_steer_value').value
        self.boost_duration = self.get_parameter('boost_duration_sec').value
        self.boost_cooldown = self.get_parameter('boost_cooldown_sec').value

        self.get_logger().info(f"Motor command send interval: {self.min_send_interval}s")
        self.get_logger().info(f"Compass/RPM read period: {self.compass_read_timer_period}s")
        # ... (остальные логи параметров, если хотите их добавить) ...

        # --- ИЗМЕНЕНИЕ: Явное указание QoS для подписчика /cmd_vel ---
        cmd_vel_subscriber_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Должно совпадать с Reliability издателя
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,    # Должно совпадать с Durability издателя
            depth=10 # Глубина очереди подписчика
        )
        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            cmd_vel_subscriber_qos) # <--- ИСПОЛЬЗУЕМ ЯВНЫЙ QoS
        # --- КОНЕЦ ИЗМЕНЕНИЯ QoS ---
        
        self.heading_publisher = self.create_publisher(
            Float32,
            '/robot/heading', 
            10) 
        self.get_logger().info("Publisher for /robot/heading created.")

        self.compass_rpm_timer = self.create_timer(
            self.compass_read_timer_period,
            self.read_compass_rpm_callback)
        self.get_logger().info(f"Timer for reading compass/RPM started (period: {self.compass_read_timer_period}s).")

        self.last_motor_cmd_sent_time = 0.0 
        self.last_i2c_motor_send_time = 0
        
        self.latest_l_rpm = 0                               
        self.latest_r_rpm = 0                               
        self.last_commanded_linear_x = 0.0                  
        self.last_commanded_angular_z = 0.0                 
        self.stuck_timer_start_time = None                  
        self.is_boosting = False                            
        self.boost_end_time = 0.0                           
        self.last_boost_finish_time = 0.0                   
        
        self.get_logger().info('Hoverboard control node initialized')

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().debug(f"cmd_vel_callback received: L.X={msg.linear.x:.2f} A.Z={msg.angular.z:.2f}")
        
        self.last_commanded_linear_x = msg.linear.x
        self.last_commanded_angular_z = msg.angular.z

        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available, cannot send motor command.", throttle_duration_sec=5.0)
            return

        current_time = time.time()
        if current_time - self.last_i2c_motor_send_time < self.min_send_interval:
            self.get_logger().debug(f"Motor command throttled. Time since last: {current_time - self.last_i2c_motor_send_time:.3f}s < interval: {self.min_send_interval:.3f}s")
            return
        
        self.last_i2c_motor_send_time = current_time 

        calculated_speed = int(msg.linear.x * 250)
        calculated_steer = int(max(-90, min(90, msg.angular.z * 90)))
        calculated_speed = max(-125, min(125, calculated_speed))
        
        command_packed_bytes = struct.pack('>hh', calculated_steer, calculated_speed) 
        i2c_payload = [0x01, len(command_packed_bytes)] + list(command_packed_bytes)

        try:
            write_transaction = i2c_msg.write(self.i2c_address, i2c_payload)
            self.i2c_bus.i2c_rdwr(write_transaction)
            self.last_motor_cmd_sent_time = time.time()
            self.get_logger().debug(f'Sent to Arduino (motors) via i2c_rdwr: steer={calculated_steer}, speed={calculated_speed}')
        except OSError as e: 
            error_message = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
            self.get_logger().error(f'I2C write error (motors): {error_message}')
            self.get_logger().error(f'Failed to send motor command due to I2C error: steer={calculated_steer}, speed={calculated_speed}')
        except Exception as e: 
            self.get_logger().error(f'Generic error during I2C write (motors): {type(e).__name__} - {e}')
            self.get_logger().error(f'Context for generic error: steer={calculated_steer}, speed={calculated_speed}')

    def read_compass_rpm_callback(self):
        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available, cannot read compass/rpm.", throttle_duration_sec=5.0)
            return

        current_time = time.time()
        if current_time - self.last_motor_cmd_sent_time < self.compass_read_quiet_period_after_motor_cmd:
            self.get_logger().debug(f"Skipping compass/rpm read: motor command sent recently "
                                    f"({current_time - self.last_motor_cmd_sent_time:.3f}s ago). "
                                    f"Quiet period: {self.compass_read_quiet_period_after_motor_cmd:.3f}s")
            return

        try:
            bytes_to_read = 6 
            read_transaction = i2c_msg.read(self.i2c_address, bytes_to_read)
            self.i2c_bus.i2c_rdwr(read_transaction)
            data_from_i2c = bytes(read_transaction) 

            if len(data_from_i2c) == bytes_to_read:
                heading_word_times_10, l_rpm, r_rpm = struct.unpack('>hhh', data_from_i2c)
                
                current_heading_degrees = float(heading_word_times_10) / 10.0
                heading_radians = math.radians(current_heading_degrees)
                self.latest_l_rpm = l_rpm
                self.latest_r_rpm = r_rpm
                
                self.get_logger().info(f"Received from Arduino: H_deg={current_heading_degrees:.1f}, "
                                       f"L_RPM={self.latest_l_rpm}, R_RPM={self.latest_r_rpm}")

                heading_msg = Float32()
                heading_msg.data = heading_radians
                self.heading_publisher.publish(heading_msg)
            else:
                self.get_logger().error(f"I2C read error (data): Expected {bytes_to_read} bytes, got {len(data_from_i2c)}", 
                                       throttle_duration_sec=5.0)
        except OSError as e: 
             error_message = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
             if hasattr(e, 'errno') and e.errno == 121: 
                self.get_logger().warn(f"I2C read error (data): Remote I/O error (errno 121). Arduino busy? Details: {error_message}", throttle_duration_sec=2.0)
             else:
                self.get_logger().error(f"I2C read error (data) OSError: {error_message}", throttle_duration_sec=5.0)
        except struct.error as e: 
            self.get_logger().error(f"Failed to unpack I2C data (struct.error): {e}. "
                                    f"Received {len(data_from_i2c)} bytes: {data_from_i2c.hex() if isinstance(data_from_i2c, bytes) else data_from_i2c}")
        except Exception as e:
            self.get_logger().error(f"Generic I2C read error (data): {type(e).__name__} - {e}", throttle_duration_sec=5.0)

        current_time_for_stuck_check = time.time() # Обновляем время для логики ниже

        if self.is_boosting: 
            if current_time_for_stuck_check >= self.boost_end_time:
                self.get_logger().info("Boost sequence finished.")
                self.is_boosting = False
                self.last_boost_finish_time = current_time_for_stuck_check 
            return 

        if current_time_for_stuck_check - self.last_boost_finish_time < self.boost_cooldown:
            self.get_logger().debug(f"In boost cooldown. Remaining: {self.boost_cooldown - (current_time_for_stuck_check - self.last_boost_finish_time):.1f}s", throttle_duration_sec=1.0)
            return 

        is_commanded_to_move = abs(self.last_commanded_linear_x) >= self.stuck_lin_vel_thresh
        is_wheels_stuck = abs(self.latest_l_rpm) < self.stuck_rpm_thresh and \
                          abs(self.latest_r_rpm) < self.stuck_rpm_thresh

        if is_commanded_to_move and is_wheels_stuck:
            if self.stuck_timer_start_time is None: 
                self.stuck_timer_start_time = current_time_for_stuck_check
                self.get_logger().info(f"Potential stuck condition: Commanded LinX={self.last_commanded_linear_x:.2f}, "
                                       f"but L_RPM={self.latest_l_rpm}, R_RPM={self.latest_r_rpm}. Monitoring...")
            elif current_time_for_stuck_check - self.stuck_timer_start_time >= self.stuck_duration_thresh:
                self.get_logger().warn(f"STUCK CONFIRMED: Commanded LinX={self.last_commanded_linear_x:.2f}, "
                                       f"L_RPM={self.latest_l_rpm}, R_RPM={self.latest_r_rpm} for "
                                       f"{current_time_for_stuck_check - self.stuck_timer_start_time:.1f}s. (Boost action to be implemented next)")
                self.stuck_timer_start_time = None 
        else:
            if self.stuck_timer_start_time is not None:
                self.get_logger().info("Stuck condition resolved or command to move stopped.")
            self.stuck_timer_start_time = None 

    def destroy_node(self):
        self.get_logger().info("Shutting down Hoverboard Control Node...")
        if hasattr(self, 'compass_rpm_timer') and self.compass_rpm_timer:
            self.compass_rpm_timer.cancel()
            self.get_logger().info("Compass/RPM timer cancelled.")

        if self.i2c_bus:
            try:
                self.get_logger().info("Sending stop command to motors before exiting.")
                stop_command_bytes = struct.pack('>hh', 0, 0)
                i2c_stop_payload = [0x01, len(stop_command_bytes)] + list(stop_command_bytes)
                write_stop_transaction = i2c_msg.write(self.i2c_address, i2c_stop_payload)
                self.i2c_bus.i2c_rdwr(write_stop_transaction)
            except Exception as e:
                self.get_logger().error(f"Failed to send stop command on destroy: {e}")
            
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
                 print("HoverboardControlNode could not be fully created (e.g. pre-super init error or RPi specific).", file=sys.stderr)
            if rclpy.ok():
                rclpy.shutdown()
            return 

    except KeyboardInterrupt:
        if node_creation_successful and node: 
            node.get_logger().info('Hoverboard control node stopped by user (KeyboardInterrupt).')
    except Exception as e:
        log_target = node.get_logger() if node_creation_successful and node else None
        if log_target:
            log_target.fatal(f"Unhandled exception: {e}", exc_info=True) 
        else:
            print(f"Unhandled critical exception during node main: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    finally:
        if node: 
            if rclpy.ok(): 
                 node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()