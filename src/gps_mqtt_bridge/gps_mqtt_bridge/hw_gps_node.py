#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import pynmea2

class HardwareGPSNode(Node):
    def __init__(self):
        super().__init__('hardware_gps_node')
        
        # Параметры
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200) # Установлена правильная скорость по умолчанию
        self.declare_parameter('frame_id', 'robot_gps_antenna')
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publisher для координат тележки
        self.robot_publisher_ = self.create_publisher(NavSatFix, '/robot/gps/fix', 10)
        
        # Открываем serial порт
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Successfully opened serial port: {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            rclpy.shutdown()
            return

        # --- ИЗМЕНЕНИЕ: Устанавливаем таймер на 0.5 секунды (2 Гц) ---
        self.timer = self.create_timer(0.5, self.read_and_publish_gps)
        self.get_logger().info("GPS node initialized. Reading and publishing at ~2 Hz.")

    def read_and_publish_gps(self):
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()
            self.get_logger().info(f"Raw GPS data: {line}") # <--- ДОБАВЛЕННАЯ СТРОКА
            # Ищем GNGGA или GPGGA, так как они содержат все нужные нам данные
            if line.startswith(('$GNGGA', '$GPGGA')): 
                msg = pynmea2.parse(line)
                # Проверяем, что есть фиксация и валидные данные
                if msg.is_valid and msg.gps_qual > 0 and hasattr(msg, 'latitude') and msg.latitude != 0.0:
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.header.frame_id = self.frame_id
                    
                    gps_msg.status.status = NavSatStatus.STATUS_FIX
                    gps_msg.status.service = NavSatStatus.SERVICE_GPS
                    
                    gps_msg.latitude = msg.latitude
                    gps_msg.longitude = msg.longitude
                    gps_msg.altitude = msg.altitude if hasattr(msg, 'altitude') else 0.0
                    
                    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    
                    self.robot_publisher_.publish(gps_msg)
                    self.get_logger().info(f"Published Hardware GPS (Robot): Lat {gps_msg.latitude:.6f}, Lon {gps_msg.longitude:.6f}",
                                           throttle_duration_sec=2.0)
        except pynmea2.ParseError as e:
            self.get_logger().warn(f"Failed to parse NMEA sentence: {e}", throttle_duration_sec=5.0)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareGPSNode()
    if hasattr(node, 'ser') and node.ser:
        rclpy.spin(node)
    # Уничтожение узла будет обработано rclpy.spin, если оно завершится корректно
    # node.destroy_node() # Часто избыточно
    rclpy.shutdown()

if __name__ == '__main__':
    main()