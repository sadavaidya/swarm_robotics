"""Central clearing of robot-computed bids; exact minimum for the small PAIR auction."""

from itertools import permutations
import math


def allocate(slot_ids, bid_table, incumbent=None):
    """Keep healthy committed members; minimize total bid for remaining slots.

    No identity preferences except lexicographic tie-breaking for equal cost.
    Requiring a complete solution prevents partial movement when undersupplied.
    """
    incumbent = incumbent or {}
    if len(bid_table) < len(slot_ids):
        return {}
    kept = {slot: robot for slot, robot in incumbent.items()
            if slot in slot_ids and robot in bid_table}
    if len(set(kept.values())) != len(kept):
        raise ValueError('A robot cannot occupy two slots')
    missing = [slot for slot in slot_ids if slot not in kept]
    candidates = sorted(set(bid_table) - set(kept.values()))
    best = None
    for robots in permutations(candidates, len(missing)):
        costs = [bid_table[robot].get(slot, math.inf) for robot, slot in zip(robots, missing)]
        if not all(math.isfinite(cost) and cost >= 0 for cost in costs):
            continue
        key = (sum(costs), robots)
        if best is None or key < best[0]:
            best = key, dict(kept, **dict(zip(missing, robots)))
    return best[1] if best else {}
