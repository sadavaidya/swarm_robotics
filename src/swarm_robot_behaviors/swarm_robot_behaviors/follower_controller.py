"""One configurable follower implementation for every link in the chain."""

from geometry_msgs.msg import Vector3Stamped

from .control import proportional_command, relative_target
from .controller_base import ControllerBase, run


class FollowerController(ControllerBase):
    """Pursue the preceding robot using common-world ground-truth poses."""

    def __init__(self):
        super().__init__('follower_controller', 'robot_2')
        self.target = self.setting('target_namespace', 'robot_1').strip('/')
        if self.target == self.robot:
            raise ValueError('A follower cannot follow itself')
        self.spacing = self.positive('desired_follow_distance', 0.6)
        self.error_pub = self.create_publisher(
            Vector3Stamped, '/' + self.robot + '/behavior/error', 10)
        self.start([self.robot, self.target])
        self.get_logger().info(self.robot + ' follows ' + self.target)

    def nominal_command(self, own, now):
        target = self.poses.get(self.target, now)
        if target is None:
            return (0.0, 0.0), 'waiting_for_target'
        distance, bearing = relative_target(own, target)
        error = Vector3Stamped()
        error.header.stamp = self.get_clock().now().to_msg()
        error.header.frame_id = self.world_frame
        error.vector.x, error.vector.y = distance, bearing
        error.vector.z = distance - self.spacing
        self.error_pub.publish(error)
        command = proportional_command(
            distance, bearing, self.spacing, self.k_distance, self.k_heading,
            self.max_linear, self.max_angular, self.distance_deadband,
            self.heading_deadband, self.turn_angle)
        return command, 'following'


def main(args=None):
    run(FollowerController, args)
