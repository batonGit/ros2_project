#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import smbus2
from smbus2 import i2c_msg # Для i2c_rdwr
import struct
import time
import math
import sys 
import os # Добавлен для os.strerror

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
            self.get_logger().error(f"I2C bus {self.i2c_bus_number} not found. Ensure I2C is enabled on Raspberry Pi (sudo raspi-config). Node will not function correctly.")
            return 
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus {self.i2c_bus_number}: {e}. Node will not function correctly.")
            return

        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10)
        
        self.heading_publisher = self.create_publisher(
            Float32,
            '/robot/heading', 
            10)
        self.get_logger().info("Publisher for /robot/heading created.")

        # Периоды из ваших предыдущих успешных тестов или настроек
        self.compass_read_timer_period = 0.5 # Пример: 2 Гц
        self.min_send_interval = 0.05      # Пример: 20 Гц макс для моторов (было 0.05 в логе)
        
        self.compass_timer = self.create_timer(
            self.compass_read_timer_period,
            self.read_and_publish_heading_callback)
        self.get_logger().info(f"Timer for reading compass started (period: {self.compass_read_timer_period}s).")

        self.last_send_time = 0
        self.get_logger().info(f"Hoverboard control node initialized (motor send interval: {self.min_send_interval}s).")

    def cmd_vel_callback(self, msg):
        self.get_logger().debug(f"cmd_vel_callback received: L.X={msg.linear.x:.2f} A.Z={msg.angular.z:.2f}")

        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available, cannot send motor command.", throttle_duration_sec=5.0)
            return

        current_time = time.time()
        if current_time - self.last_send_time < self.min_send_interval:
            self.get_logger().debug(f"Motor command throttled. Time since last: {current_time - self.last_send_time:.3f}s < interval: {self.min_send_interval:.3f}s")
            return
        self.last_send_time = current_time

        # Используем имена 'steer' и 'speed' для вычисленных значений
        calculated_speed = int(msg.linear.x * 250)
        calculated_steer = int(max(-90, min(90, msg.angular.z * 90)))
        calculated_speed = max(-125, min(125, calculated_speed))
        
        command_packed_bytes = struct.pack('>hh', calculated_steer, calculated_speed)
        
        i2c_payload = [0x01, len(command_packed_bytes)] + list(command_packed_bytes)

        try:
            write_transaction = i2c_msg.write(self.i2c_address, i2c_payload)
            self.i2c_bus.i2c_rdwr(write_transaction)
            
            # Используем правильные имена переменных 'calculated_steer' и 'calculated_speed' в логировании
            self.get_logger().debug(f'Sent to Arduino (motors) via i2c_rdwr: steer={calculated_steer}, speed={calculated_speed}')
            
        except OSError as e: 
            error_message = f"[Errno {e.errno}] {os.strerror(e.errno)}" if hasattr(e, "errno") else str(e)
            self.get_logger().error(f'I2C write error (motors): {error_message}')
            # Используем правильные имена переменных в логировании ошибки
            self.get_logger().error(f'Failed to send motor command due to I2C error: steer={calculated_steer}, speed={calculated_speed}')
        except Exception as e: 
            self.get_logger().error(f'Generic error during I2C write (motors): {type(e).__name__} - {e}')
            # Используем правильные имена переменных в логировании ошибки
            self.get_logger().error(f'Context for generic error: steer={calculated_steer}, speed={calculated_speed}')


    def read_and_publish_heading_callback(self):
        if not self.i2c_bus:
            self.get_logger().warn("I2C bus not available, cannot read compass.", throttle_duration_sec=5.0)
            return

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
                self.get_logger().error(f"I2C read error (compass): Remote I/O error (errno 121). Check Arduino slave response and I2C wiring. Details: {error_message}", throttle_duration_sec=5.0)
             else:
                self.get_logger().error(f"I2C read error (compass) OSError: {error_message}", throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Generic I2C read error (compass): {type(e).__name__} - {e}", throttle_duration_sec=5.0)


    def destroy_node(self):
        self.get_logger().info("Shutting down Hoverboard Control Node...")
        if hasattr(self, 'compass_timer') and self.compass_timer:
            self.compass_timer.cancel()
            self.get_logger().info("Compass timer cancelled.")

        if self.i2c_bus:
            try:
                self.get_logger().info("Sending stop command to motors before exiting.")
                stop_command_bytes = struct.pack('>hh', 0, 0)
                # Для отправки команды стоп используем тот же механизм, что и в cmd_vel_callback
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
    node_creation_successful = False # Флаг для корректной обработки в finally
    try:
        node = HoverboardControlNode()
        if node.i2c_bus: 
            node_creation_successful = True
            rclpy.spin(node)
        else:
            # Лог об ошибке инициализации I2C уже должен быть в __init__
            if node: # Если объект узла был создан, но шина не открылась
                 node.get_logger().fatal("I2C bus was not initialized in __init__. Node did not spin.")
            else: # Редкий случай, если даже super().__init__ не отработал
                 print("HoverboardControlNode could not be fully created, I2C bus likely an issue.", file=sys.stderr)
            
    except KeyboardInterrupt:
        if node_creation_successful and node: 
            node.get_logger().info('Hoverboard control node stopped by user (KeyboardInterrupt).')
    except Exception as e:
        # Логируем, если узел был успешно создан, иначе просто печатаем
        log_target = node.get_logger() if node_creation_successful and node else None
        if log_target:
            log_target.fatal(f"Unhandled exception: {e}", exc_info=True) # exc_info для traceback
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