"""Stand-in for the gesture-recognition model in the separate repository.

It publishes the same ``SignalCommand`` messages that the real model will, so
the rest of the pipeline can be built and run before that repository is ready.
Delete this node once the real publisher is wired in — nothing else depends on
it.
"""

import rclpy
from rclpy.node import Node

from signal_robot_msgs.msg import SignalCommand

COMMANDS = {
    'stop': SignalCommand.STOP,
    'forward': SignalCommand.FORWARD,
    'left': SignalCommand.TURN_LEFT,
    'right': SignalCommand.TURN_RIGHT,
}


class MockSignalPublisher(Node):
    """Walk through a scripted list of commands, holding each one for a while."""

    def __init__(self) -> None:
        super().__init__('mock_signal_publisher')

        self.declare_parameter('sequence', ['forward', 'left', 'forward', 'right', 'stop'])
        self.declare_parameter('hold_duration', 3.0)
        self.declare_parameter('confidence', 0.9)
        self.declare_parameter('publish_rate', 10.0)

        self._sequence = self.get_parameter('sequence').value
        self._hold_duration = self.get_parameter('hold_duration').value
        self._confidence = self.get_parameter('confidence').value
        publish_rate = self.get_parameter('publish_rate').value

        unknown = [name for name in self._sequence if name not in COMMANDS]
        if unknown:
            raise ValueError(
                f'unknown command name(s) {unknown}; expected one of {sorted(COMMANDS)}'
            )

        self._index = 0
        self._elapsed = 0.0
        self._period = 1.0 / publish_rate

        self._publisher = self.create_publisher(SignalCommand, 'signal_command', 10)
        self.create_timer(self._period, self.publish_command)

        self.get_logger().info(
            f'mock publisher started — cycling {self._sequence} '
            f'every {self._hold_duration}s'
        )

    def publish_command(self) -> None:
        name = self._sequence[self._index]

        msg = SignalCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = COMMANDS[name]
        msg.confidence = float(self._confidence)
        self._publisher.publish(msg)

        self._elapsed += self._period
        if self._elapsed >= self._hold_duration:
            self._elapsed = 0.0
            self._index = (self._index + 1) % len(self._sequence)
            self.get_logger().info(f'-> {self._sequence[self._index]}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockSignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
