"""Geometric coverage and asynchronous auction agreement without an auctioneer."""
import copy
from itertools import combinations
import math
import random

import pytest
from swarm_robot_shape_formation.auction import Auction
from swarm_robot_shape_formation.shapes import make_shape, port, edges


@pytest.mark.parametrize('name', ['LINE', 'SQUARE', 'CIRCLE'])
def test_shapes_have_ten_safe_anonymous_roles(name):
    slots = make_shape(name)
    assert len(slots) == len({s['id'] for s in slots}) == 10
    assert all(math.isfinite(v) for s in slots for v in s['target'])
    assert min(math.dist(a['target'][:2], b['target'][:2])
               for a, b in combinations(slots, 2)) >= .15
    assert not any('robot' in s for s in slots)


@pytest.mark.parametrize('name', ['LINE', 'SQUARE', 'CIRCLE'])
def test_every_requested_connection_has_coincident_ports(name):
    slots = make_shape(name)
    connections = edges(10, name != 'LINE')
    assert len(connections) == (9 if name == 'LINE' else 10)
    for a, b in connections:
        assert math.dist(port(slots[a]['target']), port(slots[b]['target'], False)) < 1e-9


def test_square_has_equal_width_and_height_and_chamfered_corners():
    points = [port(s['target']) for s in make_shape('SQUARE')]
    width = max(p[0] for p in points)-min(p[0] for p in points)
    height = max(p[1] for p in points)-min(p[1] for p in points)
    assert width == pytest.approx(height)
    assert width == pytest.approx(.22*(2+1/math.sqrt(2)))


def test_spacing_cannot_disconnect_physical_ports():
    with pytest.raises(ValueError):
        make_shape('LINE', spacing=.5)


@pytest.mark.parametrize('name', ['HEXAGON', 'PAIR'])
def test_unsupported_shapes_are_rejected(name):
    with pytest.raises(ValueError):
        make_shape(name)


@pytest.mark.parametrize('seed', range(5))
def test_delayed_ring_peer_communication_reaches_unique_consensus(seed):
    rng = random.Random(seed)
    names = [f'r{i}' for i in range(10)]
    # Equal preferences create many conflicts; all scores stay fixed per task.
    agents = [Auction(n, names, [100-j+rng.random()*.1 for j in range(10)]) for n in names]
    messages = []
    for tick in range(2000):
        for i, agent in enumerate(agents):
            if rng.random() < .7:
                agent.bid()
                for neighbor in ((i-1) % 10, (i+1) % 10):
                    messages.append((tick+rng.randint(1, 7), neighbor,
                                     copy.deepcopy(agent.winners)))
        remaining = []
        rng.shuffle(messages)
        for due, i, table in messages:
            if due <= tick:
                agents[i].merge(table)
            else:
                remaining.append((due, i, table))
        messages = remaining
        if all(a.complete for a in agents) and len({a.digest for a in agents}) == 1:
            break
    assert all(a.complete for a in agents)
    assert len({a.digest for a in agents}) == 1
    assert {a.role for a in agents} == set(range(10))


def test_bad_peer_bids_cannot_corrupt_winners():
    agent = Auction('a', ['a', 'b'], [1., 2.])
    agent.merge([{'owner': 'stranger', 'bid': 100.}, {'owner': 'b', 'bid': math.nan}])
    assert all(w['owner'] is None for w in agent.winners)


def test_peer_freshness_handles_clock_delivery_race_without_accepting_stale_data():
    from swarm_robot_shape_formation.peer_data import fresh_peer

    assert fresh_peer(10.1, 10., 1.8)
    assert not fresh_peer(100., 10., 1.8)
    assert not fresh_peer(7., 10., 1.8)


def test_peer_sample_is_cached_until_the_receiver_clock_catches_up():
    from types import SimpleNamespace
    from swarm_robot_docking.protocol import encode
    from swarm_robot_shape_formation.peer_data import fresh_peer
    from swarm_robot_shape_formation.shape_agent import ShapeAgent

    agent = SimpleNamespace(names=['a', 'b'], robot='a', epoch='task', round=0, peers={})
    msg = encode(dict(robot='b', epoch='task', round=0, stamp=10.6, healthy=True))
    ShapeAgent.on_peer(agent, msg)
    assert 'b' in agent.peers
    assert not fresh_peer(agent.peers['b']['stamp'], 10., 1.8)
    assert fresh_peer(agent.peers['b']['stamp'], 10.5, 1.8)
