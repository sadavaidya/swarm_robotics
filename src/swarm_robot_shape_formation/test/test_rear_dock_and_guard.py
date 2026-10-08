"""Geometry compatibility and independent stop-on-missing-command behavior."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Twist
import pytest
import xacro
from swarm_robot_shape_formation.velocity_guard import VelocityGuard


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_rear_dock_is_prefixed_and_optional(enabled):
    source = Path(__file__).resolve().parents[2]
    path = source/'swarm_robot_description/urdf/swarm_robot.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(path), mappings={
        'rear_docking': enabled, 'frame_prefix': 'test/', 'robot_namespace': '/test'}).toxml())
    assert robot.find("link[@name='test/front_dock_frame']") is not None
    rear = robot.find("joint[@name='test/rear_dock_frame_joint']")
    if enabled == 'false':
        assert rear is None
    else:
        assert float(rear.find('origin').get('xyz').split()[0]) == -.006
        assert float(rear.find('origin').get('rpy').split()[2]) == pytest.approx(3.141592653589793)
        assert robot.find("link[@name='test/rear_dock_link']/collision/geometry/box") is not None


def test_watchdog_stops_even_when_ros_time_does_not_advance(monkeypatch):
    command = Twist()
    command.linear.x = .15
    node = SimpleNamespace(timeout=.5, last=10., command=command, cmd_pub=Mock())
    monkeypatch.setattr('swarm_robot_shape_formation.velocity_guard.time.monotonic', lambda: 10.1)
    VelocityGuard.tick(node)
    assert node.cmd_pub.publish.call_args.args[0].linear.x == .15
    monkeypatch.setattr('swarm_robot_shape_formation.velocity_guard.time.monotonic', lambda: 10.6)
    VelocityGuard.tick(node)
    stopped = node.cmd_pub.publish.call_args.args[0]
    assert stopped.linear.x == stopped.angular.z == 0.


def test_formation_ports_match_the_connected_template_reach():
    from swarm_robot_shape_formation.shapes import DOCK_REACH
    source = Path(__file__).resolve().parents[2]
    path = source/'swarm_robot_description/urdf/swarm_robot.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(path), mappings={
        'formation_docking': 'true', 'frame_prefix': 'test/'}).toxml())
    for side, sign in [('front', 1), ('rear', -1)]:
        joint = robot.find(f"joint[@name='test/{side}_dock_frame_joint']")
        assert float(joint.find('origin').get('xyz').split()[0]) == sign*DOCK_REACH
        assert robot.find(f"link[@name='test/{side}_dock_link']/visual") is not None
        assert robot.find("link[@name='test/base_link']/collision") is not None
