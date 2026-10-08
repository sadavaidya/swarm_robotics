"""Pair capacity and collection must not prevent otherwise feasible assembly."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from swarm_robot_docking.rendezvous import rendezvous_offers
from swarm_robot_docking.shape_library import bid_cost
from swarm_robot_docking.swarm_allocator import SwarmAllocator


def coordinator(goal=0):
    return SimpleNamespace(
        resetting=False, goal=goal, input_pair_count=goal, notice='',
        get_logger=lambda: Mock(), publish_shape=Mock(), tick=Mock())


@pytest.mark.parametrize('count', [3, 4, 100])
def test_oversized_request_still_requests_two_pairs(count):
    node = coordinator()
    SwarmAllocator.request(node, count)
    assert node.goal == 2 and node.input_pair_count == count
    assert f'{count} pairs not possible' in node.notice
    node.publish_shape.assert_called_once()


def test_repeated_oversized_request_preserves_two_pair_goal():
    node = coordinator(goal=2)
    SwarmAllocator.request(node, 3)
    SwarmAllocator.request(node, 3)
    assert node.goal == 2 and node.input_pair_count == 3


@pytest.mark.parametrize('count', [0, -1])
def test_nonpositive_request_does_not_start_or_cancel_assembly(count):
    node = coordinator(goal=1)
    SwarmAllocator.request(node, count)
    assert node.goal == 1
    node.publish_shape.assert_not_called()


def test_reset_hold_cannot_be_bypassed_by_oversized_command():
    node = coordinator()
    node.resetting = True
    SwarmAllocator.request(node, 3)
    assert node.goal == 0 and node.notice.startswith('RESET_IN_PROGRESS')


def test_transient_health_change_does_not_discard_valid_bids():
    poses = {'a': [-1., 0., 0.], 'b': [-1., 1., 0.],
             'c': [1., 0., 0.], 'd': [1., 1., 0.], 'e': [3., 3., 0.]}
    offers = rendezvous_offers(poses, .062, .012, .3)
    # e temporarily drops out. The remaining four already have valid bids.
    active = {name: {'pose': pose} for name, pose in poses.items() if name != 'e'}
    bids = {name: {slot['id']: bid_cost(s['pose'], slot, .08)
                   for offer in offers for slot in offer['slots']}
            for name, s in active.items()}
    node = SimpleNamespace(
        pairs=[], goal=2, occupied=lambda: set(), generation=0, bid_wait=.6,
        clearance=.7, pending={'names': tuple(poses), 'offers': offers, 'stamp': 0.},
        bids=lambda unused: bids, get_logger=lambda: Mock(), publish_shape=Mock(),
        start_round=Mock(side_effect=AssertionError('Discarded already valid bids')))
    SwarmAllocator.plan(node, active, .8)
    assert len(node.pairs) == 2
    assert {n for p in node.pairs for n in p['members'].values()} == set(active)


def test_brief_pose_gap_reserves_member_but_cannot_authorize_motion():
    last = dict(stamp=10., healthy=True, pose=[0., 0., 0.], state='FINAL_APPROACH')
    node = SimpleNamespace(timeout=1.8, last_healthy={'a': last},
                           robots={'a': dict(stamp=10.5, healthy=False, pose=None)})
    active = SwarmAllocator.active_states(node, 10.6)
    assert 'a' in active and not active['a']['healthy']
    assert active['a']['dock'] is None and active['a']['state'] == 'WAITING_FOR_FRESH_POSE'
    assert SwarmAllocator.active_states(node, 11.81) == {}


def test_pose_gap_preserves_docked_pair():
    pair = dict(members={'A': 'a', 'B': 'b'}, slots=[{'id': 'A'}, {'id': 'B'}],
                docked=True, allow_final=True)
    active = {'a': dict(healthy=False), 'b': dict(healthy=True)}
    SwarmAllocator.progress(SimpleNamespace(), pair, active, 10.)
    assert pair['docked'] and pair['status'] == 'DOCKED'


def test_pose_gap_cancels_final_approach_permission():
    pair = dict(members={'A': 'a', 'B': 'b'}, slots=[{'id': 'A'}, {'id': 'B'}],
                docked=False, allow_final=True, close_since=10.)
    active = {'a': dict(healthy=False), 'b': dict(healthy=True)}
    SwarmAllocator.progress(SimpleNamespace(), pair, active, 10.)
    assert not pair['allow_final'] and pair['close_since'] is None
