#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus2
import struct
import time

class HoverboardControlNode(Node):
    def __init__(self):
        super().__init__('hoverboard_control_node')
        self.i2c_bus = smbus2.SMBus(1)
        self.i2c_address = 0x08
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.last_send_time = 0
        self.min_send_interval = 0.1  # 100 мс между сообщениями
        self.get_logger().info('Hoverboard control node started')

    def cmd_vel_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_send_time < self.min_send_interval:
            return
        self.last_send_time = current_time

        speed = int(msg.linear.x * 250)
        steer = int(max(-90, min(90, msg.angular.z * 90)))
        speed = max(-125, min(125, speed))
        command = struct.pack('>hh', steer, speed)
        try:
            self.i2c_bus.write_block_data(self.i2c_address, 0, list(command))
            self.get_logger().info(f'Sent: steer={steer}, speed={speed}')
        except Exception as e:
            self.get_logger().error(f'I2C error: {str(e)}')
            self.get_logger().error(f'Failed to send: steer={steer}, speed={speed}')

    def destroy_node(self):
        self.i2c_bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
