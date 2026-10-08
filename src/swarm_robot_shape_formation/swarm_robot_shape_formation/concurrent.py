"""Local collision prediction; no fleet-wide single-mover token."""
import math


def project(pose, v, w, dt):
    # Differential-drive axis is 20 mm ahead of the reported body origin.
    x, y, yaw = pose
    ax, ay = x+.02*math.cos(yaw), y+.02*math.sin(yaw)
    heading = yaw+w*dt
    if abs(w) < 1e-6:
        ax += v*dt*math.cos(yaw)
        ay += v*dt*math.sin(yaw)
    else:
        ax += v/w*(math.sin(heading)-math.sin(yaw))
        ay -= v/w*(math.cos(heading)-math.cos(yaw))
    return [ax-.02*math.cos(heading), ay-.02*math.sin(heading), heading]


def safe_command(robot, pose, v, w, peers, assembling):
    """Bounded look-ahead against peers and arena walls, slowing before contact.

    Peer commands are a short-horizon estimate, not a reservation. Both robots
    brake on predicted conflict; independent A* replanning routes around a peer
    that has stopped. Near intended docking neighbors, low speeds allow contact.
    """
    clearance = .142 if assembling else .27
    for scale in (1., .5, .2, 0.):
        valid = True
        for dt in (.15, .3, .5, .8):
            future = project(pose, v*scale, w*scale, dt)
            if max(abs(future[0]), abs(future[1])) > 4.72:
                valid = False
                break
            for peer in peers.values():
                p = peer['pose']
                pv, pw = peer.get('command', [0., 0.])
                predicted = project(p, pv, pw, dt)
                # Also check a stopped peer: its watchdog may brake immediately.
                distance = min(math.dist(future[:2], predicted[:2]),
                               math.dist(future[:2], p[:2]))
                initial = math.dist(pose[:2], p[:2])
                if distance < clearance and distance < initial-.001:
                    valid = False
                    break
            if not valid:
                break
        if valid:
            return v*scale, w*scale
    return 0., 0.
