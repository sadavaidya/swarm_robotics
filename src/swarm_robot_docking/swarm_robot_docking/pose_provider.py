"""Ground-truth pose access with a bounded cache for odometry/TF arrival races."""

from rclpy.clock import JumpThreshold
from rclpy.duration import Duration

from swarm_robot_behaviors.pose_provider import PoseProvider as WorldPoseProvider
from swarm_robot_behaviors.pose_provider import stamp_seconds

from .protocol import fresh


class PoseProvider(WorldPoseProvider):
    """Reuse the common-world transform helper, retaining only fresh actual samples.

    Gazebo publishes odom and its matching TF separately. Their arrival order
    varies between robots. A single missing exact-time transform must not make
    a healthy robot drop out of the auction. No pose is extrapolated, and the
    original sample timestamp remains subject to the configured freshness limit.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.successful = {}
        node = args[0] if args else kwargs['node']
        self.clock_jump = node.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=False), post_callback=self.on_clock_rewind)

    def on_clock_rewind(self, jump):
        # Humble's TF Buffer does not automatically clear on a ROS clock rewind.
        # Drop samples from the previous timeline before accepting new transforms.
        self.clear()
        self.buffer.clear()

    def clear(self):
        super().clear()
        self.successful.clear()

    def get(self, name, now):
        pose = super().get(name, now)
        if pose is not None:
            stamp = stamp_seconds(self.messages[name].header.stamp)
            self.successful[name] = (pose, stamp)
            return pose
        cached = self.successful.get(name)
        if cached and fresh(cached[1], now, self.timeout):
            return cached[0]
        return None
