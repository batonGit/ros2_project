#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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

        # --- ОПРЕДЕЛИТЬ И ИСПОЛЬЗОВАТЬ QoS ДЛЯ CMD_VEL ---
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Должен совпадать с publisher'ом
            history=HistoryPolicy.KEEP_LAST,      # Должен совпадать с publisher'ом
            durability=DurabilityPolicy.VOLATILE,   # Должен совпадать с publisher'ом
            depth=10 # Глубина подписчика может быть больше или равна глубине издателя
        )
        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            cmd_vel_qos)
        
        self.heading_publisher = self.create_publisher(
            Float32,
            '/robot/heading', 
            10)
        self.get_logger().info("Publisher for /robot/heading created.")

        # --- ИЗМЕНЕННЫЕ И НОВЫЕ ПАРАМЕТРЫ ---
        # Период чтения компаса (например, 1 раз в секунду = 1 Гц)
        self.compass_read_timer_period = 1.0  # Было 0.5
        # Минимальный интервал между отправкой команд моторам (100 мс = 10 Гц, как в Arduino TIME_SEND)
        self.min_send_interval = 0.1 # Было 0.05
        
        # НОВОЕ: Тихий период для чтения компаса после отправки команды мотору
        self.compass_read_quiet_period_after_motor_cmd = self.min_send_interval * 0.5 # Например, половина интервала отправки моторам

        self.compass_timer = self.create_timer(
            self.compass_read_timer_period,
            self.read_and_publish_heading_callback)
        self.get_logger().info(f"Timer for reading compass started (period: {self.compass_read_timer_period}s).")

        self.last_motor_cmd_sent_time = 0.0 # НОВОЕ: Время последней отправленной команды мотору
        self.last_i2c_motor_send_time = 0 # Переименовано из self.last_send_time для ясности
        
        self.get_logger().info(f"Hoverboard control node initialized (motor send interval: {self.min_send_interval}s).")

    def cmd_vel_callback(self, msg):
        self.get_logger().debug(f"cmd_vel_callback received: L.X={msg.linear.x:.2f} A.Z={msg.angular.z:.2f}")
        
        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available, cannot send motor command.", throttle_duration_sec=5.0)
            return

        current_time = time.time()
        if current_time - self.last_i2c_motor_send_time < self.min_send_interval: # Используем переименованную переменную
            self.get_logger().debug(f"Motor command throttled. Time since last: {current_time - self.last_i2c_motor_send_time:.3f}s < interval: {self.min_send_interval:.3f}s")
            return
        self.last_i2c_motor_send_time = current_time # Используем переименованную переменную

        calculated_speed = int(msg.linear.x * 250)
        calculated_steer = int(max(-90, min(90, msg.angular.z * 90)))
        calculated_speed = max(-125, min(125, calculated_speed))
        
        command_packed_bytes = struct.pack('>hh', calculated_steer, calculated_speed) 
        i2c_payload = [0x01, len(command_packed_bytes)] + list(command_packed_bytes)

        try:
            write_transaction = i2c_msg.write(self.i2c_address, i2c_payload)
            self.i2c_bus.i2c_rdwr(write_transaction)
            
            self.last_motor_cmd_sent_time = time.time() # <--- НОВОЕ: Фиксируем время успешной отправки
            
            self.get_logger().debug(f'Sent to Arduino (motors) via i2c_rdwr: steer={calculated_steer}, speed={calculated_speed}')
        except OSError as e: 
            error_message = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
            self.get_logger().error(f'I2C write error (motors): {error_message}')
            self.get_logger().error(f'Failed to send motor command due to I2C error: steer={calculated_steer}, speed={calculated_speed}')
        except Exception as e: 
            self.get_logger().error(f'Generic error during I2C write (motors): {type(e).__name__} - {e}')
            self.get_logger().error(f'Context for generic error: steer={calculated_steer}, speed={calculated_speed}')

    def read_and_publish_heading_callback(self):
        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available, cannot read compass.", throttle_duration_sec=5.0)
            return

        # --- НОВОЕ: Проверка "тихого периода" после отправки команды мотору ---
        current_time = time.time()
        if current_time - self.last_motor_cmd_sent_time < self.compass_read_quiet_period_after_motor_cmd:
            self.get_logger().debug(f"Skipping compass read: motor command sent recently "
                                    f"({current_time - self.last_motor_cmd_sent_time:.3f}s ago). "
                                    f"Quiet period: {self.compass_read_quiet_period_after_motor_cmd:.3f}s")
            return
        # --- КОНЕЦ НОВОЙ ПРОВЕРКИ ---

        try:
            read_transaction = i2c_msg.read(self.i2c_address, 2)
            self.i2c_bus.i2c_rdwr(read_transaction)
            data_bytes = list(read_transaction)
            
            if len(data_bytes) == 2:
                msb = data_bytes[0]
                lsb = data_bytes[1]
                heading_word_signed = (msb << 8) | lsb 
                current_heading_degrees = float(heading_word_signed) / 10.0
                heading_radians = math.radians(current_heading_degrees) 

                heading_msg = Float32()
                heading_msg.data = heading_radians
                self.heading_publisher.publish(heading_msg)
                self.get_logger().debug(f"Published heading: {current_heading_degrees:.1f} deg ({heading_radians:.3f} rad)")
            else:
                self.get_logger().error(f"I2C read error (compass): Expected 2 bytes, got {len(data_bytes)}", throttle_duration_sec=5.0)

        except OSError as e:
             error_message = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
             if hasattr(e, 'errno') and e.errno == 121: 
                self.get_logger().warn(f"I2C read error (compass): Remote I/O error (errno 121). Arduino busy? Details: {error_message}", throttle_duration_sec=2.0) # Изменено на WARN для редких случаев
             else:
                self.get_logger().error(f"I2C read error (compass) OSError: {error_message}", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Generic I2C read error (compass): {type(e).__name__} - {e}", throttle_duration_sec=5.0)

    # ... (destroy_node и main остаются как в предыдущей версии, где я исправлял NameError) ...
    # ВАЖНО: Скопируйте destroy_node и main из моего предыдущего ответа, где были исправлены NameError
    # и улучшена обработка исключений в main. Я не буду их здесь повторять для краткости,
    # но они должны быть такими же, как в том полном коде, который я вам дал после исправления NameError.
    # Если нужно, я могу предоставить полный файл снова.

    def destroy_node(self): # Копирую актуальную версию destroy_node
        self.get_logger().info("Shutting down Hoverboard Control Node...")
        if hasattr(self, 'compass_timer') and self.compass_timer:
            self.compass_timer.cancel()
            self.get_logger().info("Compass timer cancelled.")

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

def main(args=None): # Копирую актуальную версию main
    rclpy.init(args=args)
    node = None
    node_creation_successful = False 
    try:
        node = HoverboardControlNode()
        if node.i2c_bus: 
            node_creation_successful = True
            rclpy.spin(node)
        else:
            if node: 
                 node.get_logger().fatal("I2C bus was not initialized in __init__. Node did not spin.")
            else: 
                 print("HoverboardControlNode could not be fully created, I2C bus likely an issue.", file=sys.stderr)
            
    except KeyboardInterrupt:
        if node_creation_successful and node: 
            node.get_logger().info('Hoverboard control node stopped by user (KeyboardInterrupt).')
    except Exception as e:
        log_target = node.get_logger() if node_creation_successful and node else None
        if log_target:
            log_target.fatal(f"Unhandled exception: {e}", exc_info=True) 
        else:
            print(f"Unhandled critical exception: {e}", file=sys.stderr)
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