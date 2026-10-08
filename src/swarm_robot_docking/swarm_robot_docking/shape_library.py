"""Robot-ID-free abstract shapes, expressed as base poses plus staging poses."""

import math

from .protocol import wrap


def pair_slots(origin, dock_offset, gap, staging_distance):
    """Opposite front faces end gap metres apart; no robot identity is encoded."""
    x, y, yaw = origin
    half = dock_offset + gap / 2
    slots = []
    for name, side, angle in [('slot_A', -1, 0), ('slot_B', 1, math.pi)]:
        target = [x + side * half * math.cos(yaw),
                  y + side * half * math.sin(yaw), wrap(yaw + angle)]
        staging = [target[0] - staging_distance * math.cos(target[2]),
                   target[1] - staging_distance * math.sin(target[2]), target[2]]
        slots.append({'id': name, 'target': target, 'staging': staging})
    return slots


def make_shape(name, origin, dock_offset, gap, staging_distance):
    if name != 'PAIR':
        raise ValueError('Only PAIR is implemented')
    return pair_slots(origin, dock_offset, gap, staging_distance)


def bid_cost(pose, slot, heading_weight):
    target = slot['target']
    return (math.hypot(target[0] - pose[0], target[1] - pose[1]) +
            heading_weight * abs(wrap(target[2] - pose[2])))


def docking_errors(a, b):
    """Two outward-facing front frames must be antiparallel, not parallel."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    return {'separation': math.hypot(dx, dy),
            'lateral': abs(-math.sin(a[2]) * dx + math.cos(a[2]) * dy),
            'yaw_error': abs(wrap(b[2] - a[2] - math.pi))}
