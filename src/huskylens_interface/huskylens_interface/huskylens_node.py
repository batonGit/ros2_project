#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial # для работы с последовательным портом (pip install pyserial)
import time

class HuskyLensNode(Node):
    def __init__(self):
        super().__init__('huskylens_node')

        # --- Параметры ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # Укажите ваш порт
        self.declare_parameter('baud_rate', 9600) # Или другая скорость HuskyLens (проверьте документацию)

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.ser = None
        try:
            self.ser = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=1 # Таймаут чтения в секундах
            )
            self.get_logger().info(f"Successfully opened serial port: {self.serial_port_name} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port_name}: {e}")
            # Завершаем работу узла, если порт не открылся
            rclpy.shutdown() 
            return
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred opening serial port: {e}")
            rclpy.shutdown()
            return

        # Таймер для чтения данных с порта
        self.read_timer_period = 0.1 # Читаем каждые 100 мс (10 Гц)
        self.read_timer = self.create_timer(self.read_timer_period, self.read_serial_data)
        self.get_logger().info("HuskyLens node initialized and reading serial data.")

    def read_serial_data(self):
        if self.ser and self.ser.is_open:
            self.get_logger().info(f"in_waiting: {self.ser.in_waiting}")
            if self.ser.in_waiting > 0:
                try:
                    data_bytes = self.ser.read(self.ser.in_waiting)
                    self.get_logger().info(f"Received from HuskyLens: {data_bytes}")
                except Exception as e:
                    self.get_logger().error(f"Error processing serial data: {e}")
            else:
                self.get_logger().warn("No data available on serial port.")
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HuskyLensNode()
    if node.ser and node.ser.is_open: # Только если порт успешно открыт
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("HuskyLens node stopped by user.")
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
    else:
        # Лог об ошибке открытия порта уже был в __init__
        # rclpy.shutdown() также был вызван в __init__ в случае ошибки
        print("HuskyLensNode could not initialize serial port. Exiting.", file=sys.stderr)
        if rclpy.ok(): # На всякий случай, если shutdown не был вызван
             rclpy.shutdown()


if __name__ == '__main__':
    main()