"""Common simulated-time command loop, stale-data stop and range override."""

import math
import signal
import time

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .control import RangeGuard, scan_minimum
from .pose_provider import fresh, PoseProvider, stamp_seconds


class ControllerBase(Node):
    """Shared safety for both leader and followers; one cmd_vel owner per robot."""

    def __init__(self, name, default_namespace):
        super().__init__(name)
        inferred_namespace = self.get_namespace().strip('/') or default_namespace
        self.robot = self.setting('robot_namespace', inferred_namespace).strip('/')
        self.world_frame = self.setting('world_frame', 'world')
        self.max_linear = self.positive('max_linear_speed', 0.20)
        self.max_angular = self.positive('max_angular_speed', 0.8)
        self.k_distance = self.positive('k_distance', 0.6)
        self.k_heading = self.positive('k_heading', 1.5)
        self.distance_deadband = self.positive('distance_deadband', 0.03)
        self.heading_deadband = self.positive('heading_deadband', 0.03)
        self.turn_angle = self.positive('turn_in_place_angle', 0.8)
        self.timeout = self.positive('data_timeout', 0.5)
        self.rate = self.positive('control_rate', 20.0)
        stop = self.positive('stop_distance', 0.25)
        clear = self.positive('clear_distance', 0.35)
        if clear <= stop:
            raise ValueError('clear_distance must exceed stop_distance')
        turn = min(self.max_angular, self.positive('avoid_turn_speed', 0.5))
        self.guard = RangeGuard(stop, clear, turn, self.positive('min_turn_time', 0.7))
        self.enabled = self.setting('enabled', True)
        self.scan = None
        self.scan_stamp = None
        self.last_tick = None
        self.state = None
        self.command_pub = self.create_publisher(Twist, '/' + self.robot + '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/' + self.robot + '/behavior/state', 10)
        self.range_sub = self.create_subscription(
            LaserScan, '/' + self.robot + '/range/front', self.on_scan, qos_profile_sensor_data)
        self.enable_service = self.create_service(SetBool, '~/set_enabled', self.set_enabled)

    def setting(self, name, default):
        return self.declare_parameter(
            name, default, ParameterDescriptor(read_only=True)).value

    def positive(self, name, default):
        value = float(self.setting(name, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + ' must be finite and positive')
        return value

    def start(self, robot_names):
        self.poses = PoseProvider(self, robot_names, self.world_frame, self.timeout)
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def on_scan(self, msg):
        if msg.header.frame_id != self.robot + '/front_range_link':
            return
        self.scan = scan_minimum(msg.ranges)
        self.scan_stamp = stamp_seconds(msg.header.stamp)

    def set_enabled(self, request, response):
        self.enabled = request.data
        self.guard.reset()
        self.publish(0.0, 0.0)
        response.success = True
        response.message = 'enabled' if self.enabled else 'disabled; zero command published'
        return response

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.command_pub.publish(msg)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_tick is not None and now < self.last_tick:
            self.poses.clear()
            self.scan = None
            self.guard.reset()
        self.last_tick = now
        command = (0.0, 0.0)
        state = 'disabled'
        if self.enabled:
            own = self.poses.get(self.robot, now)
            if (own is None or self.scan is None or self.scan_stamp is None or
                    not fresh(self.scan_stamp, now, self.timeout)):
                state = 'waiting_for_fresh_data'
            else:
                override = self.guard.command(self.scan, now)
                if override is not None:
                    command, state = override, 'avoiding'
                else:
                    command, state = self.nominal_command(own, now)
        self.publish(*command)
        self.state_pub.publish(String(data=state))
        if state != self.state:
            self.get_logger().info(state)
            self.state = state

    def nominal_command(self, own, now):
        raise NotImplementedError


def run(controller_class, args=None):
    """Try to send zero before graceful shutdown; hard kills cannot do this."""
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = controller_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(3):
                    node.publish(0.0, 0.0)
                    time.sleep(0.03)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
