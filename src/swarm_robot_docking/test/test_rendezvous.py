"""Check peer-relative meeting poses, disjoint pair selection and bid validation."""

import math
import pytest

from swarm_robot_docking.rendezvous import clear_pair_bids, rendezvous_offers
from swarm_robot_docking.shape_library import bid_cost


POSES = {'a': [-1., 0., 0.], 'b': [-1.8, .8, 0.],
         'c': [1., 0., math.pi], 'd': [1.8, .8, math.pi], 'e': [-2., -1.5, 0.]}


def offers(poses=POSES, **kwargs):
    return rendezvous_offers(poses, .062, .012, .3, **kwargs)


def bids(candidates, poses=POSES):
    return {name: {slot['id']: bid_cost(pose, slot, .08)
                   for offer in candidates for slot in offer['slots']}
            for name, pose in poses.items()}


def test_two_pairs_use_four_distinct_robots():
    candidates = offers()
    selected = clear_pair_bids(candidates, bids(candidates), 2)
    assert len(selected) == 2
    assert len({n for offer in selected for n in offer['owners'].values()}) == 4
    owners = {frozenset(o['owners'].values()) for o in selected}
    assert owners == {frozenset('ab'), frozenset('cd')}


def test_rendezvous_tracks_translation_of_robots():
    moved = {name: [p[0]+2, p[1]-3, p[2]] for name, p in POSES.items()}
    for original, translated in zip(offers(), offers(moved)):
        expected = [original['origin'][0]+2, original['origin'][1]-3, original['origin'][2]]
        assert translated['origin'] == pytest.approx(expected)
        a, b = list(original['owners'].values())
        expected = [(POSES[a][0]+POSES[b][0])/2, (POSES[a][1]+POSES[b][1])/2]
        assert original['origin'][:2] == expected


def test_completed_pair_area_excluded():
    origin = offers()[0]['origin']
    assert all(math.dist(o['origin'][:2], origin[:2]) >= .7
               for o in offers(occupied_origins=[origin]))


def test_insufficient_robots_cannot_be_reused_for_two_pairs():
    poses = dict(list(POSES.items())[:3])
    candidates = offers(poses)
    assert clear_pair_bids(candidates, bids(candidates, poses), 2) == []


def test_staging_stays_between_close_partners():
    poses = {'near': [0., 0., 0.], 'other': [.2, 0., math.pi]}
    candidate = offers(poses)[0]
    assert all(0 <= s['staging'][0] <= .2 for s in candidate['slots'])


def test_invalid_or_missing_bids_do_not_win():
    candidates = offers()
    assert clear_pair_bids(candidates, {}, 1) == []
    table = bids(candidates)
    for row in table.values():
        for slot in row:
            row[slot] = math.inf
    assert clear_pair_bids(candidates, table, 1) == []
