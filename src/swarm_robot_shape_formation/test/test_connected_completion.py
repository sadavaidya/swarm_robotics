"""Perfect poses alone are not docking; the exact fresh physical graph is required."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from swarm_robot_docking.protocol import decode
from swarm_robot_shape_formation import formation_monitor
from swarm_robot_shape_formation.formation_monitor import FormationMonitor
from swarm_robot_shape_formation.shapes import make_shape, edges


@pytest.mark.parametrize('physical,fresh,closed,expected', [
    (False, True, True, False), (True, False, True, False),
    (True, True, False, False), (True, True, True, True),
])
def test_completion_requires_current_closed_physical_graph(monkeypatch, physical, fresh,
                                                           closed, expected):
    monitor = FormationMonitor.__new__(FormationMonitor)
    monitor.names = [f'robot_{i}' for i in range(10)]
    slots = make_shape('CIRCLE')
    monitor.goal = dict(epoch='task', slots=slots, shape='CIRCLE')
    winners = [dict(owner=n) for n in monitor.names]
    monitor.peers = {n: dict(epoch='task', stamp=10., healthy=True, ready=True,
                             digest='agreed', round=0, pose=slots[i]['target'],
                             state='WAITING_FOR_DOCK', winners=winners)
                     for i, n in enumerate(monitor.names)}
    monitor.docks = dict(epoch='task', stamp=10. if fresh else 1., physical=physical,
                         bonds=[dict(a=monitor.names[a], b=monitor.names[b])
                                for a, b in edges(10, closed)])
    monitor.last_now, monitor.close_since = 9., 9.
    monitor.status_pub = SimpleNamespace(publish=Mock())
    monitor.markers = Mock()
    monkeypatch.setattr(formation_monitor, 'now_seconds', lambda _: 10.)
    monitor.tick()
    status = decode(monitor.status_pub.publish.call_args.args[0])
    assert status['complete'] is expected
    assert status['mechanical_attachment'] is expected
