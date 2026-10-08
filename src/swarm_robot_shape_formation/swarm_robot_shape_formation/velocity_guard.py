"""Independent wall-time watchdog between the agent and Gazebo's drive plugin."""
import time

from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from swarm_robot_docking.protocol import run, setting


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        self.timeout = float(setting(self, 'command_timeout', .5))
        self.last = -1.
        self.command = Twist()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, 'shape/cmd_vel', self.receive, 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.04, self.tick, clock=self.wall_clock)

    def receive(self, msg):
        self.last, self.command = time.monotonic(), msg

    def tick(self):
        fresh_command = time.monotonic()-self.last < self.timeout
        self.cmd_pub.publish(self.command if fresh_command else Twist())

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    run(VelocityGuard, args)
