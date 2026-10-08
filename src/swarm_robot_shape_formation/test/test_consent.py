"""A depth-one channel must preserve simultaneous consent for both ports."""
from swarm_robot_shape_formation.consent import connections_for


def test_both_ports_are_in_one_snapshot():
    owners = [f'r{i}' for i in range(10)]
    peers = {n: dict(arrived=True) for n in owners}
    assert connections_for(4, owners, False, peers, []) == [
        dict(a='r3', b='r4'), dict(a='r4', b='r5')]
    assert connections_for(0, owners, True, peers, []) == [
        dict(a='r0', b='r1'), dict(a='r9', b='r0')]


def test_completed_edges_and_unready_neighbors_revoke_consent():
    owners = [f'r{i}' for i in range(10)]
    peers = {n: dict(arrived=True) for n in owners}
    peers['r5']['arrived'] = False
    assert connections_for(4, owners, True, peers, [dict(a='r3', b='r4')]) == []
