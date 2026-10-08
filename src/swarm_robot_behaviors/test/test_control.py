"""Behavioral regressions for control signs, bounds, deadbands and safety."""

import math

import pytest

from swarm_robot_behaviors.control import (
    Pose2D, proportional_command, RangeGuard, relative_target, scan_minimum,
)
from swarm_robot_behaviors.pose_provider import fresh


def command(distance, bearing):
    return proportional_command(distance, bearing, 0.6, 0.6, 1.5, 0.2, 0.8, 0.03, 0.03, 0.8)


def test_relative_pose_uses_own_heading_and_wraps():
    distance, bearing = relative_target(Pose2D(2, 3, math.pi/2), Pose2D(3, 3, 0))
    assert distance == 1
    assert bearing == pytest.approx(-math.pi/2)
    _, bearing = relative_target(Pose2D(0, 0, -math.pi + .01), Pose2D(-1, .01, 0))
    assert abs(bearing) < .03


@pytest.mark.parametrize('distance,bearing', [(0, 0), (.6, .02), (.62, -.02)])
def test_spacing_deadband_and_no_reverse(distance, bearing):
    assert command(distance, bearing) == (0.0, 0.0)


def test_far_target_clamps_speed_and_turns_first():
    assert command(10, 0) == (.2, 0)
    assert command(10, 1.0) == (0, .8)
    assert command(10, -1.0) == (0, -.8)
    assert command(10, math.pi)[0] == 0


def test_turn_slows_translation_and_has_correct_sign():
    linear, angular = command(1.0, .4)
    assert 0 < linear < .2
    assert angular == pytest.approx(.6)


def test_range_no_return_and_invalid_values():
    assert scan_minimum([math.inf] * 3) == math.inf
    assert scan_minimum([math.nan, .2, math.inf]) == .2
    assert scan_minimum([math.nan, -1]) is None
    assert scan_minimum([]) is None
    assert scan_minimum([-math.inf, .5]) == 0


def test_safety_latch_hysteresis_and_clock_reset():
    guard = RangeGuard(.25, .35, .5, .7)
    assert guard.command(.24, 1) == (0, .5)
    assert guard.command(math.inf, 1.2) == (0, .5)
    assert guard.command(.30, 2) == (0, .5)
    assert guard.command(.36, 2.1) is None
    assert guard.command(.10, 3) == (0, .5)
    assert guard.command(math.inf, 0) is None


def test_stale_and_future_messages_are_rejected():
    assert fresh(2, 2.49, .5)
    assert not fresh(2, 2.51, .5)
    assert not fresh(2, 1, .5)
