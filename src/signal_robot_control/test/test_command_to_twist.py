"""Tests for the command-to-motion mapping.

The safety behaviour is what matters here: a robot that keeps driving on a stale
or uncertain command is the exact failure this project exists to prevent, so
those paths are tested rather than just the happy one.
"""

import pytest
import rclpy

from signal_robot_control.command_to_twist import CommandToTwist
from signal_robot_msgs.msg import SignalCommand


@pytest.fixture
def node():
    rclpy.init()
    node = CommandToTwist()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def command(kind: int, confidence: float = 0.9) -> SignalCommand:
    msg = SignalCommand()
    msg.command = kind
    msg.confidence = confidence
    return msg


def test_forward_drives_straight(node):
    node.on_command(command(SignalCommand.FORWARD))
    assert node._twist.linear.x > 0.0
    assert node._twist.angular.z == 0.0


def test_left_and_right_turn_opposite_ways(node):
    node.on_command(command(SignalCommand.TURN_LEFT))
    left = node._twist.angular.z

    node.on_command(command(SignalCommand.TURN_RIGHT))
    right = node._twist.angular.z

    assert left > 0.0
    assert right == pytest.approx(-left)


def test_stop_zeroes_velocity(node):
    node.on_command(command(SignalCommand.FORWARD))
    node.on_command(command(SignalCommand.STOP))

    assert node._twist.linear.x == 0.0
    assert node._twist.angular.z == 0.0


def test_low_confidence_is_refused(node):
    node.on_command(command(SignalCommand.FORWARD))
    node.on_command(command(SignalCommand.FORWARD, confidence=0.1))

    assert node._twist.linear.x == 0.0


def test_unknown_command_stops(node):
    node.on_command(command(SignalCommand.FORWARD))
    node.on_command(command(SignalCommand.UNKNOWN))

    assert node._twist.linear.x == 0.0


def test_stale_command_stops_the_robot(node):
    node.on_command(command(SignalCommand.FORWARD))
    assert node._twist.linear.x > 0.0

    # Pretend the perception pipeline went quiet longer ago than the timeout
    # allows, rather than sleeping through it.
    timeout = node.get_parameter('command_timeout').value
    node._last_command_time -= rclpy.duration.Duration(seconds=timeout + 1.0)

    node.publish_twist()

    assert node._twist.linear.x == 0.0
