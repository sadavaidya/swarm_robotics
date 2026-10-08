"""Dock-connected planar templates; each robot is a 0.22 m connector segment."""
import math

from swarm_robot_docking.protocol import wrap

DOCK_REACH = .11


def port(pose, front=True):
    distance = DOCK_REACH if front else -DOCK_REACH
    return [pose[0]+distance*math.cos(pose[2]), pose[1]+distance*math.sin(pose[2])]


def edges(count, closed):
    return [(i, (i+1) % count) for i in range(count if closed else count-1)]


def make_shape(name, count=10, spacing=.22, origin=(0., 0., 0.)):
    name = name.strip().upper()
    if name not in ('LINE', 'SQUARE', 'CIRCLE'):
        raise ValueError('Supported shapes: LINE, SQUARE, CIRCLE')
    if count != 10 or not math.isfinite(spacing) or abs(spacing-2*DOCK_REACH) > 1e-6:
        raise ValueError('Dock geometry requires ten robots and spacing 0.22 m')
    if not all(math.isfinite(v) for v in origin):
        raise ValueError('Origin must be finite')
    x, y, yaw = origin
    if name == 'LINE':
        points = [((i-(count-1)/2)*spacing, 0., 0.) for i in range(count)]
    else:
        if name == 'CIRCLE':
            radius = DOCK_REACH/math.sin(math.pi/count)
            vertices = [(radius*math.cos(2*math.pi*i/count),
                         radius*math.sin(2*math.pi*i/count)) for i in range(count)]
        else:
            # Ten equal-length robots cannot tile four equal straight sides.
            # Chamfer two opposite corners: eight side segments + two diagonals.
            trim = spacing/math.sqrt(2)
            half = (2*spacing+trim)/2
            corners = [(-half+trim, -half), (half, -half), (half, half-trim),
                       (half-trim, half), (-half, half), (-half, -half+trim)]
            vertices = []
            for i, a in enumerate(corners):
                b = corners[(i+1) % len(corners)]
                pieces = 1 if i in (2, 5) else 2
                vertices.extend((a[0]+(b[0]-a[0])*j/pieces,
                                 a[1]+(b[1]-a[1])*j/pieces) for j in range(pieces))
        points = []
        for i, a in enumerate(vertices):
            b = vertices[(i+1) % count]
            points.append(((a[0]+b[0])/2, (a[1]+b[1])/2,
                           math.atan2(b[1]-a[1], b[0]-a[0])))
    slots = []
    for i, (px, py, angle) in enumerate(points):
        target = [x+px*math.cos(yaw)-py*math.sin(yaw),
                  y+px*math.sin(yaw)+py*math.cos(yaw), wrap(angle+yaw)]
        if abs(target[0]) > 3.5 or abs(target[1]) > 3.5:
            raise ValueError('Shape must fit inside the arena with approach space')
        # Distributed agents first reach separated staging poses concurrently.
        # Assembly then approaches each role from outside the connected outline.
        if name == 'LINE':
            sx, sy = px*1.8, 1.2
        else:
            scale = 1.5/math.hypot(px, py)
            sx, sy = px*scale, py*scale
        staging = [x+sx*math.cos(yaw)-sy*math.sin(yaw),
                   y+sx*math.sin(yaw)+sy*math.cos(yaw), target[2]]
        slots.append(dict(id=f'slot_{i:02}', target=target, staging=staging))
    return slots
