"""Refresh each robot's URDF fixed frames for late-joining inspection tools."""
import math
from xml.etree import ElementTree

from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from swarm_robot_docking.protocol import LATCHED


def fixed_transforms(description):
    transforms = []
    for joint in ElementTree.fromstring(description).findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        transform = TransformStamped()
        transform.header.frame_id = joint.find('parent').get('link')
        transform.child_frame_id = joint.find('child').get('link')
        origin = joint.find('origin')
        xyz = [0., 0., 0.] if origin is None else [
            float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [0., 0., 0.] if origin is None else [
            float(v) for v in origin.get('rpy', '0 0 0').split()]
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll/2), math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        q = transform.transform.rotation
        q.x, q.y = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy
        q.z, q.w = cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
        transforms.append(transform)
    return transforms


class StaticFrames:
    def __init__(self, node, robot):
        self.node = node
        self.root = TransformStamped()
        self.root.header.frame_id, self.root.child_frame_id = 'world', robot+'/odom'
        self.root.transform.rotation.w = 1.
        self.transforms = []
        self.broadcaster = StaticTransformBroadcaster(node)
        self.subscription = node.create_subscription(
            String, 'robot_description', self.on_description, LATCHED)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(1., self.publish, clock=self.wall_clock)
        self.publish()

    def on_description(self, msg):
        try:
            self.transforms = fixed_transforms(msg.data)
            self.publish()
        except (ElementTree.ParseError, ValueError, AttributeError, IndexError) as error:
            self.node.get_logger().error('Invalid fixed-frame description: '+str(error))

    def publish(self):
        transforms = [self.root]+self.transforms
        stamp = self.node.get_clock().now().to_msg()
        for transform in transforms:
            transform.header.stamp = stamp
        self.broadcaster.sendTransform(transforms)
