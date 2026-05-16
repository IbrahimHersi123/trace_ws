#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped


IMAGE_WIDTH    = 640
IMAGE_CENTER_X = IMAGE_WIDTH / 2.0


class BallOrientNode(Node):

    def __init__(self):
        super().__init__('ball_orient_node')

        # Parameters
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('dead_zone', 20.0)
        self.declare_parameter('max_angular_z', 0.5)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('toggle_button', 0)   # X button on PS4

        self.kp_              = self.get_parameter('kp').get_parameter_value().double_value
        self.dead_zone_       = self.get_parameter('dead_zone').get_parameter_value().double_value
        self.max_angular_z_   = self.get_parameter('max_angular_z').get_parameter_value().double_value
        self.watchdog_timeout_= self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.toggle_button_   = self.get_parameter('toggle_button').get_parameter_value().integer_value

        # State
        self.enabled_          = False
        self.prev_button_state_= 0
        self.ball_count_       = 0
        self.cx_values_        = []
        self.last_msg_time_    = None

        # Subscribers
        self.create_subscription(Int32, '/ball_count', self.ball_count_callback, 10)
        self.create_subscription(Float32MultiArray, '/ball_positions', self.ball_positions_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publisher
        self.cmd_vel_pub_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Watchdog timer — runs at 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Ball orient node started. X button to toggle.')

    def joy_callback(self, msg: Joy):
        if self.toggle_button_ >= len(msg.buttons):
            return

        current = msg.buttons[self.toggle_button_]

        # Detect rising edge only — button just pressed
        if current == 1 and self.prev_button_state_ == 0:
            self.enabled_ = not self.enabled_
            self.get_logger().info(
                f'Ball orientation -> {"ENABLED" if self.enabled_ else "DISABLED"}'
            )
            if not self.enabled_:
                self.publish_zero()

        self.prev_button_state_ = current

    def ball_count_callback(self, msg: Int32):
        self.ball_count_ = msg.data
        self.last_msg_time_ = self.get_clock().now()

    def ball_positions_callback(self, msg: Float32MultiArray):
        self.cx_values_ = list(msg.data)

    def control_loop(self):
        if not self.enabled_:
            return

        # Watchdog — no message received within timeout
        if self.last_msg_time_ is None:
            self.publish_zero()
            return

        elapsed = (self.get_clock().now() - self.last_msg_time_).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout_:
            self.get_logger().warn('Watchdog triggered — no ball tracker message received.')
            self.publish_zero()
            return

        # No ball detected
        if self.ball_count_ == 0 or len(self.cx_values_) == 0:
            self.publish_zero()
            return

        # For now use the first (and only) ball
        # This is where multi-ball selection logic will go later
        ball_cx = self.cx_values_[0]

        pixel_error = ball_cx - IMAGE_CENTER_X

        # Dead zone check
        if abs(pixel_error) < self.dead_zone_:
            self.publish_zero()
            return

        # Proportional control
        angular_z = -self.kp_ * pixel_error

        # Clamp
        angular_z = max(-self.max_angular_z_, min(self.max_angular_z_, angular_z))

        self.publish_cmd(angular_z)

    def publish_cmd(self, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub_.publish(msg)

    def publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)


def main():
    rclpy.init()
    node = BallOrientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()