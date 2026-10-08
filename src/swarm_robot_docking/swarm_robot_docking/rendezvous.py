"""Peer-relative rendezvous offers and exact disjoint-pair bid clearing."""

from itertools import combinations
import math

from .shape_library import pair_slots


def rendezvous_offers(poses, dock_offset, gap, staging_distance, occupied_origins=(),
                      clearance=.7):
    """The robot-independent PAIR template is placed between proposed partners."""
    offers = []
    for index, (a, b) in enumerate(combinations(sorted(poses), 2)):
        pa, pb = poses[a], poses[b]
        distance = math.dist(pa[:2], pb[:2])
        if distance < 2 * dock_offset + gap:
            continue
        origin = [(pa[0]+pb[0])/2, (pa[1]+pb[1])/2,
                  math.atan2(pb[1]-pa[1], pb[0]-pa[0])]
        if any(math.dist(origin[:2], old[:2]) < clearance for old in occupied_origins):
            continue
        # Never require a close pair to drive backwards to far staging poses.
        staging = min(staging_distance, max(0.0, distance/2 - dock_offset - gap/2 - .02))
        slots = pair_slots(origin, dock_offset, gap, staging)
        offer_id = f'offer_{index}'
        for slot, suffix in zip(slots, ('A', 'B')):
            slot['id'] = offer_id + '/' + suffix
        offers.append({'id': offer_id, 'origin': origin, 'slots': slots,
                       'owners': {slots[0]['id']: a, slots[1]['id']: b}})
    return offers


def clear_pair_bids(offers, bids, count, clearance=.7):
    """Minimize total robot-side bids; every selected robot belongs to one pair."""
    valid = []
    for offer in offers:
        costs = [bids.get(robot, {}).get(slot, math.inf)
                 for slot, robot in offer['owners'].items()]
        if all(isinstance(c, (int, float)) and math.isfinite(c) and c >= 0 for c in costs):
            valid.append((offer, sum(costs)))
    best = None
    for selection in combinations(valid, count):
        robots = [robot for offer, _ in selection for robot in offer['owners'].values()]
        if len(set(robots)) != 2 * count:
            continue
        if any(math.dist(a[0]['origin'][:2], b[0]['origin'][:2]) < clearance
               for a, b in combinations(selection, 2)):
            continue
        key = (sum(cost for _, cost in selection), tuple(o['id'] for o, _ in selection))
        if best is None or key < best[0]:
            best = key, [offer for offer, _ in selection]
    return best[1] if best else []
