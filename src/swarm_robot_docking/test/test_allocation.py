"""Regress anonymous bidding, slot geometry, failure recovery and success checks."""

import math

import pytest

from swarm_robot_docking.auction import allocate
from swarm_robot_docking.protocol import fresh
from swarm_robot_docking.shape_library import bid_cost, docking_errors, pair_slots


def test_shape_is_robot_independent_and_faces_meet():
    slots = pair_slots([0, 0, 0], .062, .012, .3)
    assert {s['id'] for s in slots} == {'slot_A', 'slot_B'}
    assert all(set(s) == {'id', 'target', 'staging'} for s in slots)
    assert slots[0]['target'] == pytest.approx([-.068, 0, 0])
    assert slots[1]['target'] == pytest.approx([.068, 0, math.pi])
    assert slots[0]['staging'][0] == pytest.approx(-.368)


def test_rotated_translated_shape_preserves_mating_geometry():
    slots = pair_slots([1, 2, math.pi/2], .062, .012, .3)
    poses = [s['target'] for s in slots]
    assert poses[0][:2] == pytest.approx([1, 1.932])
    assert poses[1][:2] == pytest.approx([1, 2.068])
    docks = [[p[0]+.062*math.cos(p[2]), p[1]+.062*math.sin(p[2]), p[2]] for p in poses]
    errors = docking_errors(*docks)
    assert errors['separation'] == pytest.approx(.012)
    assert errors['yaw_error'] < 1e-10
    assert errors['lateral'] < 1e-10


def test_swapping_positions_changes_assignment():
    slots = pair_slots([0, 0, 0], .062, .012, .3)
    positions = {'far': [-3, 2, 0], 'near_left': [-.4, 0, 0],
                 'near_right': [.4, 0, math.pi]}
    bids = {name: {s['id']: bid_cost(p, s, .08) for s in slots}
            for name, p in positions.items()}
    assert allocate(['slot_A', 'slot_B'], bids) == {
        'slot_A': 'near_left', 'slot_B': 'near_right'}
    bids['far'], bids['near_left'] = bids['near_left'], bids['far']
    assert allocate(['slot_A', 'slot_B'], bids)['slot_A'] == 'far'


def test_complete_solution_beats_greedy_choice():
    bids = {'alpha': {'A': 1, 'B': 2}, 'beta': {'A': 2, 'B': 100}}
    assert allocate(['A', 'B'], bids) == {'A': 'beta', 'B': 'alpha'}


def test_failed_member_is_replaced_and_healthy_member_retained():
    bids = {'survivor': {'A': 1, 'B': 1}, 'replacement': {'A': 5, 'B': 2}}
    old = {'A': 'survivor', 'B': 'failed'}
    assert allocate(['A', 'B'], bids, old) == {'A': 'survivor', 'B': 'replacement'}
    assert allocate(['A', 'B'], {'survivor': bids['survivor']}, old) == {}


def test_equal_bids_are_deterministic_and_invalid_bids_excluded():
    bids = {'b': {'A': 1, 'B': 1}, 'a': {'A': 1, 'B': 1}, 'c': {'A': math.nan}}
    assert allocate(['A', 'B'], bids) == {'A': 'a', 'B': 'b'}


def test_dock_heading_must_be_antiparallel_and_lateral_error_bounded():
    assert docking_errors([0, 0, 0], [.01, 0, 0])['yaw_error'] == pytest.approx(math.pi)
    assert docking_errors([0, 0, 0], [.01, .03, math.pi])['lateral'] > .02


def test_heartbeat_age_rejects_expired_and_future_data():
    assert fresh(10, 11.7, 1.8)
    assert not fresh(10, 11.9, 1.8)
    assert not fresh(11, 10, 1.8)
    assert not fresh(None, 10, 1.8)
