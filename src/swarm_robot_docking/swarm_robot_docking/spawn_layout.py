"""Bounded random placement for the primitive robots in the empty arena."""

import math
import random

# Conservative radius enclosing the chassis, wheels, caster and front plate
# at every heading. Center spacing also leaves room for initial rotation.
ROBOT_RADIUS = .12


def random_layout(robots, bounds, min_separation=.8, wall_clearance=.5,
                  seed=None, max_attempts=20000):
    """Sample x/y and yaw without changing identities or the input configuration.

    Bounds describe the inner wall faces, [xmin, xmax, ymin, ymax]. Fail rather
    than spawning overlapping robots when the requested packing cannot be found.
    A seed reproduces the layout; no process-global random generator is changed.
    """
    if (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
            or not math.isfinite(min_separation) or not math.isfinite(wall_clearance)):
        raise ValueError('Spawn bounds and spacing must be finite')
    if min_separation < 2*ROBOT_RADIUS or wall_clearance < ROBOT_RADIUS:
        raise ValueError('Spawn separation must be >= 0.24 m and wall clearance >= 0.12 m')
    if not isinstance(max_attempts, int) or max_attempts < 1:
        raise ValueError('Spawn max_attempts must be a positive integer')
    xmin, xmax, ymin, ymax = bounds
    xmin, xmax = xmin+wall_clearance, xmax-wall_clearance
    ymin, ymax = ymin+wall_clearance, ymax-wall_clearance
    if xmin >= xmax or ymin >= ymax:
        raise ValueError('Spawn wall clearance leaves no usable arena area')
    rng = random.Random(seed)
    layout = []
    attempts = 0
    for robot in robots:
        while attempts < max_attempts:
            attempts += 1
            x, y = rng.uniform(xmin, xmax), rng.uniform(ymin, ymax)
            if any(math.hypot(x-r['x'], y-r['y']) < min_separation for r in layout):
                continue
            layout.append(dict(robot, x=x, y=y, yaw=rng.uniform(-math.pi, math.pi)))
            break
        else:
            raise ValueError(
                f'Cannot safely place {len(robots)} robots after {max_attempts} attempts; '
                'reduce spawn spacing or increase the arena bounds')
    return layout
