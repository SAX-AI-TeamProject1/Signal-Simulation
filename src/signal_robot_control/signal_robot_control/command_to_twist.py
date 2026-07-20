"""Translate discrete signal-worker commands into robot velocity.

This is the real control logic of this repository: it consumes whatever the
gesture-recognition model emits and turns it into motion. The model itself lives
in a separate repository; here it is stood in for by ``mock_signal_publisher``.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from signal_robot_msgs.msg import SignalCommand


class CommandToTwist(Node):
    """Hold a velocity per command, and stop when the signal goes quiet."""

    def __init__(self) -> None:
        super().__init__('command_to_twist')

        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)

        self._linear_speed = self.get_parameter('linear_speed').value
        self._angular_speed = self.get_parameter('angular_speed').value
        self._confidence_threshold = self.get_parameter('confidence_threshold').value
        self._command_timeout = self.get_parameter('command_timeout').value
        publish_rate = self.get_parameter('publish_rate').value

        self._twist = Twist()
        self._last_command_time = self.get_clock().now()
        self._stopped_by_timeout = False

        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(SignalCommand, 'signal_command', self.on_command, 10)
        self.create_timer(1.0 / publish_rate, self.publish_twist)

        self.get_logger().info('command_to_twist ready — waiting for /signal_command')

    def on_command(self, msg: SignalCommand) -> None:
        self._last_command_time = self.get_clock().now()
        self._stopped_by_timeout = False

        # A misread gesture moves a cargo robot through a factory, so anything
        # the model is unsure about is treated as no command at all.
        if msg.confidence < self._confidence_threshold:
            self.get_logger().warn(
                f'ignoring low-confidence command {msg.command} '
                f'({msg.confidence:.2f} < {self._confidence_threshold:.2f}) — stopping'
            )
            self._twist = Twist()
            return

        self._twist = self.twist_for(msg.command)

    def twist_for(self, command: int) -> Twist:
        twist = Twist()
        if command == SignalCommand.FORWARD:
            twist.linear.x = self._linear_speed
        elif command == SignalCommand.TURN_LEFT:
            twist.angular.z = self._angular_speed
        elif command == SignalCommand.TURN_RIGHT:
            twist.angular.z = -self._angular_speed
        elif command != SignalCommand.STOP:
            self.get_logger().warn(f'unknown command {command} — stopping')
        return twist

    def publish_twist(self) -> None:
        # The robot must not coast on a stale command: if the perception
        # pipeline dies or the worker leaves the frame, the last message would
        # otherwise keep it driving forever.
        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds / 1e9
        if elapsed > self._command_timeout:
            if not self._stopped_by_timeout:
                self.get_logger().warn(
                    f'no command for {elapsed:.1f}s — stopping'
                )
                self._stopped_by_timeout = True
            self._twist = Twist()

        self._publisher.publish(self._twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
