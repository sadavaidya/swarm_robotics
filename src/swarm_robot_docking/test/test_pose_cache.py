"""The TF arrival cache must never extend a sample's freshness window."""

from nav_msgs.msg import Odometry
from swarm_robot_behaviors.control import Pose2D
from swarm_robot_behaviors.pose_provider import PoseProvider as WorldPoseProvider

from swarm_robot_docking.pose_provider import PoseProvider


def test_cached_transform_expires_at_original_odom_timestamp(monkeypatch):
    provider = PoseProvider.__new__(PoseProvider)
    provider.timeout = .7
    provider.successful = {}
    msg = Odometry()
    msg.header.stamp.sec = 10
    provider.messages = {'unit': msg}
    pose = Pose2D(1.0, 2.0, .3)
    monkeypatch.setattr(WorldPoseProvider, 'get', lambda *args: pose)
    assert provider.get('unit', 10.1) == pose
    monkeypatch.setattr(WorldPoseProvider, 'get', lambda *args: None)
    assert provider.get('unit', 10.6) == pose
    assert provider.get('unit', 10.71) is None
    assert provider.get('unit', 9.0) is None
    provider.clear()
    assert provider.successful == provider.messages == {}


def test_clock_rewind_clears_old_tf_and_keeps_static_reference():
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_ros import Buffer

    provider = PoseProvider.__new__(PoseProvider)
    provider.messages = {'unit': Odometry()}
    provider.successful = {'unit': (Pose2D(0., 0., 0.), 100.)}
    provider.buffer = Buffer()
    static = TransformStamped()
    static.header.frame_id, static.child_frame_id = 'world', 'unit/odom'
    static.transform.rotation.w = 1.
    provider.buffer.set_transform_static(static, 'test')
    moving = TransformStamped()
    moving.header.frame_id, moving.child_frame_id = 'unit/odom', 'unit/base'
    moving.header.stamp.sec = 100
    moving.transform.rotation.w = 1.
    provider.buffer.set_transform(moving, 'test')
    provider.on_clock_rewind(None)
    assert provider.successful == provider.messages == {}
    moving.header.stamp.sec = 1
    moving.transform.translation.x = 2.
    provider.buffer.set_transform(moving, 'test')
    tf = provider.buffer.lookup_transform('world', 'unit/base', Time(seconds=1))
    assert tf.transform.translation.x == 2.
