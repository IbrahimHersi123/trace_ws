#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import serial.serialutil


class BldcNode(Node):

    def __init__(self):
        super().__init__('bldc_node')

        self.declare_parameter('port', '/dev/ttyBLDC')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('button', 7)  # R1 by default

        port   = self.get_parameter('port').get_parameter_value().string_value
        baud   = self.get_parameter('baud').get_parameter_value().integer_value
        self.button_index_ = self.get_parameter('button').get_parameter_value().integer_value

        self.serial_ = None
        self._open_serial(port, baud)

        self.enabled_ = False

        self.sub_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info(f'BLDC node started on {port}, toggle button index: {self.button_index_}')

    def _open_serial(self, port, baud):
        try:
            self.serial_ = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Opened serial port {port}')
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Could not open {port}: {e}')
            self.serial_ = None

    def joy_callback(self, msg: Joy):
        if self.button_index_ >= len(msg.buttons):
            return

        pressed = msg.buttons[self.button_index_] == 1

        # Only act on state changes
        if pressed == self.enabled_:
            return

        self.enabled_ = pressed
        self._send(pressed)

    def _send(self, enable: bool):
        if self.serial_ is None or not self.serial_.is_open:
            self.get_logger().warn('Serial port not open, ignoring command')
            return

        cmd = b'1\n' if enable else b'0\n'
        try:
            self.serial_.write(cmd)
            self.get_logger().info(f'BLDC motors -> {"ON" if enable else "OFF"}')
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def destroy_node(self):
        if self.serial_ and self.serial_.is_open:
            try:
                self.serial_.write(b'0\n')
            except Exception:
                pass
            self.serial_.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = BldcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()