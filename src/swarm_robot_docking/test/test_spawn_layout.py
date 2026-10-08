"""Random layouts must preserve spacing, wall clearance and reproducibility."""

from itertools import combinations
import math

import pytest

from swarm_robot_docking.spawn_layout import random_layout
from swarm_robot_docking.rendezvous import clear_pair_bids, rendezvous_offers
from swarm_robot_docking.shape_library import bid_cost


ROBOTS = [dict(id=f'robot_{i}', x=0., y=0., yaw=0.) for i in range(1, 6)]
BOUNDS = [-5., 5., -5., 5.]


def test_random_layouts_are_safe_at_every_heading():
    for seed in range(100):
        layout = random_layout(ROBOTS, BOUNDS, seed=seed)
        assert [r['id'] for r in layout] == [r['id'] for r in ROBOTS]
        assert all(-4.5 <= r['x'] <= 4.5 and -4.5 <= r['y'] <= 4.5
                   and -math.pi <= r['yaw'] <= math.pi for r in layout)
        assert all(math.hypot(a['x']-b['x'], a['y']-b['y']) >= .8
                   for a, b in combinations(layout, 2))
    assert all(r['x'] == r['y'] == r['yaw'] == 0 for r in ROBOTS)


def test_seed_repeats_layout_and_different_seeds_change_it():
    first = random_layout(ROBOTS, BOUNDS, seed=17)
    assert first == random_layout(ROBOTS, BOUNDS, seed=17)
    assert first != random_layout(ROBOTS, BOUNDS, seed=18)


def test_impossible_packing_fails_instead_of_overlapping():
    with pytest.raises(ValueError, match='Cannot safely place'):
        random_layout(ROBOTS[:2], [-.2, .2, -.2, .2], min_separation=.3,
                      wall_clearance=.12, seed=0, max_attempts=100)


@pytest.mark.parametrize('kwargs', [
    {'min_separation': .1}, {'wall_clearance': .01},
    {'min_separation': math.nan}, {'wall_clearance': math.inf},
    {'wall_clearance': 5.}, {'max_attempts': 0},
])
def test_invalid_spacing_is_rejected(kwargs):
    with pytest.raises(ValueError):
        random_layout(ROBOTS, BOUNDS, **kwargs)


def test_random_layouts_allow_every_robot_to_be_selected():
    selected = set()
    for seed in range(20):
        layout = random_layout(ROBOTS, BOUNDS, seed=seed)
        poses = {r['id']: [r['x'], r['y'], r['yaw']] for r in layout}
        offers = rendezvous_offers(poses, .062, .012, .3)
        bids = {name: {slot['id']: bid_cost(pose, slot, .08)
                       for offer in offers for slot in offer['slots']}
                for name, pose in poses.items()}
        for offer in clear_pair_bids(offers, bids, 2):
            selected.update(offer['owners'].values())
    assert selected == {r['id'] for r in ROBOTS}
