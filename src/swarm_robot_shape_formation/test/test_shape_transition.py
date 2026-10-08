"""Stop/release barriers, latest request, cancellation and failure safety."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from std_msgs.msg import String
from swarm_robot_docking.protocol import decode
from swarm_robot_shape_formation.shape_goal import ShapeGoal
from swarm_robot_shape_formation.shapes import make_shape


@pytest.fixture
def goal(monkeypatch):
    clock = [10.]
    monkeypatch.setattr('swarm_robot_shape_formation.shape_goal.time.monotonic', lambda: clock[0])
    node = ShapeGoal.__new__(ShapeGoal)
    node.count, node.spacing, node.names = 10, .22, ['one', 'two']
    node.transition_timeout, node.resetting, node.transition = 30., False, None
    node.docks, node.docks_received = dict(bonds=[{}]), 10.
    node.states, node.commands = {}, {}
    node.current_goal = dict(epoch='old', shape='LINE', slots=make_shape('LINE'))
    node.goal_pub = SimpleNamespace(publish=Mock())
    node.result_pub = SimpleNamespace(publish=Mock())
    node.get_logger = Mock(return_value=SimpleNamespace(info=Mock(), warning=Mock()))
    node.clock = clock
    return node


def ack(goal, now):
    goal.clock[0] = now
    for name in goal.names:
        goal.states[name] = (dict(epoch=goal.current_goal['epoch'], state='TRANSITION_HOLD'), now)
        goal.commands[name] = (0., 0., now)


def release_barrier(goal):
    start = goal.clock[0]
    goal.command(String(data='CIRCLE'))
    ack(goal, start+.1)
    goal.advance_transition()
    ack(goal, start+.8)
    goal.advance_transition()
    assert goal.transition['stage'] == 'RELEASE'


def test_new_shape_waits_for_stop_then_physical_release(goal):
    goal.command(String(data='CIRCLE'))
    assert goal.current_goal['slots'] == []
    assert goal.current_goal['transition']['stage'] == 'HOLD'
    goal.advance_transition()
    assert goal.transition['stage'] == 'HOLD'
    ack(goal, 10.1)
    goal.commands['two'] = (.1, 0., 10.1)
    goal.advance_transition()
    assert goal.transition['stage'] == 'HOLD'
    ack(goal, 10.2)
    goal.advance_transition()
    ack(goal, 10.9)
    goal.advance_transition()
    assert goal.transition['stage'] == 'RELEASE'
    held_epoch = goal.current_goal['epoch']
    # Empty inventory from HOLD cannot prove that RELEASE was processed.
    goal.docks = dict(epoch=held_epoch, transition_stage='HOLD', physical=True, bonds=[])
    goal.docks_received = 11.
    ack(goal, 11.)
    goal.advance_transition()
    assert goal.current_goal['shape'] == 'TRANSITION'
    goal.docks['transition_stage'] = 'RELEASE'
    goal.advance_transition()
    assert goal.current_goal['shape'] == 'CIRCLE'
    assert goal.current_goal['reconfigure'] is True
    assert goal.current_goal['epoch'] != held_epoch
    assert goal.transition is None


@pytest.mark.parametrize('bad', ['wrong_epoch', 'nonphysical', 'old_reception', 'bonds_remain'])
def test_unverified_release_never_starts_motion(goal, bad):
    release_barrier(goal)
    goal.docks = dict(epoch=goal.current_goal['epoch'], transition_stage='RELEASE',
                      physical=True, bonds=[])
    goal.docks_received = 10.9
    if bad == 'wrong_epoch':
        goal.docks['epoch'] = 'previous'
    elif bad == 'nonphysical':
        goal.docks['physical'] = False
    elif bad == 'old_reception':
        goal.docks_received = 10.
    else:
        goal.docks['bonds'] = [{}]
    ack(goal, 10.9)
    goal.advance_transition()
    assert goal.current_goal['shape'] == 'TRANSITION'


def test_latest_valid_request_wins_without_restarting_barrier(goal):
    release_barrier(goal)
    epoch = goal.current_goal['epoch']
    goal.command(String(data='SQUARE'))
    assert goal.transition['name'] == 'SQUARE'
    assert goal.transition['stage'] == 'RELEASE'
    assert goal.current_goal['epoch'] == epoch
    goal.command(String(data='HEXAGON'))
    assert goal.transition['name'] == 'SQUARE'
    assert decode(goal.result_pub.publish.call_args.args[0])['accepted'] is False


def test_stop_and_reset_cancel_pending_transition(goal):
    release_barrier(goal)
    goal.command(String(data='STOP'))
    assert goal.transition is None and goal.current_goal['shape'] == 'STOP'
    goal.advance_transition()
    assert goal.current_goal['shape'] == 'STOP'
    release_barrier(goal)
    goal.hold(SimpleNamespace(data=True), SimpleNamespace())
    assert goal.transition is None and goal.resetting
    assert goal.current_goal['shape'] == 'IDLE'


def test_timeout_stays_held_and_can_be_retried(goal):
    goal.command(String(data='CIRCLE'))
    goal.clock[0] = 41.
    goal.advance_transition()
    assert goal.transition['stage'] == 'FAILED'
    assert goal.current_goal['slots'] == []
    goal.command(String(data='SQUARE'))
    assert goal.transition['stage'] == 'HOLD' and goal.transition['name'] == 'SQUARE'


def test_repeating_active_shape_keeps_existing_epoch(goal):
    goal.command(String(data='LINE'))
    assert goal.current_goal['epoch'] == 'old' and goal.transition is None
