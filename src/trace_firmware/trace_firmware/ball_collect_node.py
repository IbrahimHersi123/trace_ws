#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Float32MultiArray, Bool
from geometry_msgs.msg import TwistStamped
from enum import Enum, auto


IMAGE_WIDTH    = 640
IMAGE_CENTER_X = IMAGE_WIDTH / 2.0


class State(Enum):
    IDLE        = auto()
    ORIENTING   = auto()
    APPROACHING = auto()
    COLLECTED   = auto()
    NO_BALL     = auto()


class BallCollectNode(Node):

    def __init__(self):
        super().__init__('ball_collect_node')

        # Parameters
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('dead_zone', 20.0)
        self.declare_parameter('max_angular_z', 0.5)
        self.declare_parameter('approach_speed', 0.1)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('toggle_button', 2)   # Square button on PS4

        self.kp_               = self.get_parameter('kp').get_parameter_value().double_value
        self.dead_zone_        = self.get_parameter('dead_zone').get_parameter_value().double_value
        self.max_angular_z_    = self.get_parameter('max_angular_z').get_parameter_value().double_value
        self.approach_speed_   = self.get_parameter('approach_speed').get_parameter_value().double_value
        self.watchdog_timeout_ = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.toggle_button_    = self.get_parameter('toggle_button').get_parameter_value().integer_value

        # State
        self.state_             = State.IDLE
        self.prev_button_state_ = 0
        self.ball_count_        = 0
        self.cx_values_         = []
        self.last_msg_time_     = None

        # Subscribers
        self.create_subscription(Int32, '/ball_count', self.ball_count_callback, 10)
        self.create_subscription(Float32MultiArray, '/ball_positions', self.ball_positions_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publishers
        self.cmd_vel_pub_  = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.bldc_pub_     = self.create_publisher(Bool, '/bldc/enable', 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Ball collect node started. Square button to toggle.')

    # ─────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        if self.toggle_button_ >= len(msg.buttons):
            return

        current = msg.buttons[self.toggle_button_]

        # Detect rising edge only
        if current == 1 and self.prev_button_state_ == 0:
            if self.state_ == State.IDLE:
                self.state_ = State.ORIENTING
                self.get_logger().info('Ball collect -> ENABLED, entering ORIENTING')
            else:
                self._disable()

        self.prev_button_state_ = current

    def ball_count_callback(self, msg: Int32):
        self.ball_count_ = msg.data
        self.last_msg_time_ = self.get_clock().now()

    def ball_positions_callback(self, msg: Float32MultiArray):
        self.cx_values_ = list(msg.data)

    # ─────────────────────────────────────────────────────────────
    # Control loop
    # ─────────────────────────────────────────────────────────────

    def control_loop(self):
        if self.state_ == State.IDLE:
            return

        # Watchdog
        if self.last_msg_time_ is None:
            self.publish_zero()
            return

        elapsed = (self.get_clock().now() - self.last_msg_time_).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout_:
            self.get_logger().warn('Watchdog triggered — no ball tracker message received.')
            self.publish_zero()
            return

        if self.state_ == State.ORIENTING:
            self._run_orienting()

        elif self.state_ == State.APPROACHING:
            self._run_approaching()

        elif self.state_ == State.COLLECTED:
            self._run_collected()

        elif self.state_ == State.NO_BALL:
            self._run_no_ball()

    # ─────────────────────────────────────────────────────────────
    # State handlers
    # ─────────────────────────────────────────────────────────────

    def _run_orienting(self):
        if self.ball_count_ == 0 or len(self.cx_values_) == 0:
            self.get_logger().warn('Ball lost during ORIENTING, entering NO_BALL')
            self.state_ = State.NO_BALL
            self.publish_zero()
            return

        ball_cx     = self.cx_values_[0]
        pixel_error = ball_cx - IMAGE_CENTER_X

        if abs(pixel_error) < self.dead_zone_:
            self.get_logger().info('Ball centred, entering APPROACHING')
            self.state_ = State.APPROACHING
            self._set_bldc(True)
            return

        angular_z = -self.kp_ * pixel_error
        angular_z = max(-self.max_angular_z_, min(self.max_angular_z_, angular_z))
        self.publish_cmd(linear_x=0.0, angular_z=angular_z)

    def _run_approaching(self):
        # Ball collected — count dropped to zero
        if self.ball_count_ == 0:
            self.get_logger().info('Ball collected, entering COLLECTED')
            self.state_ = State.COLLECTED
            return

        ball_cx     = self.cx_values_[0]
        pixel_error = ball_cx - IMAGE_CENTER_X

        # Correct orientation while approaching
        angular_z = 0.0
        if abs(pixel_error) >= self.dead_zone_:
            angular_z = -self.kp_ * pixel_error
            angular_z = max(-self.max_angular_z_, min(self.max_angular_z_, angular_z))

        self.publish_cmd(linear_x=self.approach_speed_, angular_z=angular_z)

    def _run_collected(self):
        self.publish_zero()
        self._set_bldc(False)
        self.get_logger().info('Ball collected, stopping. Returning to IDLE.')
        self.state_ = State.IDLE

    def _run_no_ball(self):
        self.publish_zero()

        # Ball reappeared
        if self.ball_count_ > 0 and len(self.cx_values_) > 0:
            self.get_logger().info('Ball reacquired, returning to ORIENTING')
            self.state_ = State.ORIENTING

    # ─────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────

    def _disable(self):
        self.get_logger().info('Ball collect -> DISABLED, returning to IDLE')
        self.state_ = State.IDLE
        self.publish_zero()
        self._set_bldc(False)

    def _set_bldc(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.bldc_pub_.publish(msg)
        self.get_logger().info(f'BLDC -> {"ON" if enable else "OFF"}')

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub_.publish(msg)

    def publish_zero(self):
        self.publish_cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = BallCollectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node._set_bldc(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()