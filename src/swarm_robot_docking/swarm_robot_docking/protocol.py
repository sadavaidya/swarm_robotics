"""Versioned JSON envelopes over ROS String; one schema for every robot."""

import json
import math
import signal
import time

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def encode(data):
    return String(data=json.dumps(dict(data, version=1), allow_nan=False, sort_keys=True))


def decode(msg):
    try:
        value = json.loads(msg.data)
        return value if isinstance(value, dict) and value.get('version') == 1 else None
    except (ValueError, TypeError):
        return None


def now_seconds(node):
    return node.get_clock().now().nanoseconds * 1e-9


def setting(node, name, default):
    return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value


def number(node, name, default, minimum=0.0):
    value = float(setting(node, name, default))
    if not math.isfinite(value) or value <= minimum:
        raise ValueError(name + ' is outside its valid positive range')
    return value


def fresh(stamp, now, timeout):
    return isinstance(stamp, (int, float)) and 0 <= now - stamp <= timeout


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def finite_pose(pose):
    return (isinstance(pose, list) and len(pose) == 3 and
            all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose))


def run(node_class, args=None):
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok() and hasattr(node, 'stop'):
                for _ in range(3):
                    node.stop()
                    time.sleep(.03)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
