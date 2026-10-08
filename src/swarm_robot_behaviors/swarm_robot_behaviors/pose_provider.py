"""Privileged simulation pose access, isolated for future localization replacement.

The existing model has odometry_source=1, so every odom pose is in Gazebo world
coordinates. The experiment publishes identity world -> robot_N/odom links.
We still transform messages through TF rather than subtract unrelated frames.
Do NOT reuse these identity links with encoder odometry or real localization.
"""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import tf2_geometry_msgs  # noqa: F401 (register PoseStamped transforms)
from tf2_ros import Buffer, TransformException, TransformListener

from .control import Pose2D


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(stamp, now, timeout):
    return 0.0 <= now - stamp <= timeout


class PoseProvider:
    """Cache stamped odometry and expose fresh, transformed planar poses."""

    def __init__(self, node, robot_names, world_frame, timeout):
        self.world_frame = world_frame
        self.timeout = timeout
        self.messages = {}
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.subscriptions = [
            node.create_subscription(
                Odometry, '/' + name + '/odom',
                lambda msg, name=name: self.messages.__setitem__(name, msg),
                qos_profile_sensor_data)
            for name in set(robot_names)
        ]

    def clear(self):
        self.messages.clear()

    def get(self, name, now):
        msg = self.messages.get(name)
        if msg is None or not fresh(stamp_seconds(msg.header.stamp), now, self.timeout):
            return None
        if msg.header.frame_id != name + '/odom' or msg.child_frame_id != name + '/base_footprint':
            return None
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        try:
            pose = self.buffer.transform(pose, self.world_frame)
        except TransformException:
            return None
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return None
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if abs(norm - 1.0) > 0.01:
            return None
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        return Pose2D(p.x, p.y, yaw)
