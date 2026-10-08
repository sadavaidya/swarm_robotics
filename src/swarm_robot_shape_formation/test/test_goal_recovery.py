"""Repeated goal delivery must preserve allocation and completion stability."""
from types import SimpleNamespace
from unittest.mock import Mock

from swarm_robot_docking.protocol import decode, encode
from swarm_robot_shape_formation.formation_monitor import FormationMonitor
from swarm_robot_shape_formation.shape_agent import ShapeAgent
from swarm_robot_shape_formation.shape_goal import ShapeGoal


def test_goal_rebroadcast_keeps_task_identity():
    goal = ShapeGoal.__new__(ShapeGoal)
    messages = []
    goal.goal_pub = SimpleNamespace(publish=messages.append)
    goal.resetting = False
    goal.publish('LINE', [{'id': 'slot_00', 'target': [0., 0., 0.]}])
    first = decode(messages[0])
    for _ in range(3):
        goal.rebroadcast()
    assert all(decode(message) == first for message in messages)
    goal.publish('STOP', [])
    assert decode(messages[-1])['epoch'] != first['epoch']


def test_monitor_duplicate_goal_preserves_completion_window():
    monitor = FormationMonitor.__new__(FormationMonitor)
    monitor.goal = {'epoch': 'current'}
    monitor.close_since = 12.5
    monitor.on_goal(encode(dict(epoch='current', shape='CIRCLE', slots=[])))
    assert monitor.close_since == 12.5
    monitor.on_goal(encode(dict(epoch='next', shape='SQUARE', slots=[])))
    assert monitor.close_since is None


def test_agent_duplicate_goal_does_not_restart_auction():
    agent = ShapeAgent.__new__(ShapeAgent)
    agent.epoch, agent.round = 'current', 2
    agent.clear_task = Mock()
    agent.on_goal(encode(dict(epoch='current', shape='LINE', slots=[])))
    agent.clear_task.assert_not_called()
    assert agent.round == 2
    agent.on_goal(encode(dict(epoch='next', shape='LINE', slots=[])))
    agent.clear_task.assert_called_once()
    assert agent.epoch == 'next' and agent.round == 0
