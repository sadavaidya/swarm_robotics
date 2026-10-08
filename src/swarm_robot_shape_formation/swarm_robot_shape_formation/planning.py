"""Bounded obstacle-aware A* used independently by each robot."""
import heapq
import math


def segment_clear(a, b, obstacles, clearance):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = dx*dx+dy*dy
    for p in obstacles:
        t = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.
        if math.hypot(a[0]+t*dx-p[0], a[1]+t*dy-p[1]) < clearance:
            return False
    return True


def path_to(start, goal, obstacles, clearance=.35, resolution=.08, limit=18000):
    if not segment_clear(goal, goal, obstacles, clearance):
        return []
    if segment_clear(start, goal, obstacles, clearance):
        return [list(goal[:2])]
    low, high = -4.7, 4.7
    size = int((high-low)/resolution)

    def cell(p):
        return (round((p[0]-low)/resolution), round((p[1]-low)/resolution))

    source, dest = cell(start), cell(goal)

    def point(c):
        if c == source:
            return start[:2]
        if c == dest:
            return goal[:2]
        return (low+c[0]*resolution, low+c[1]*resolution)

    occupied = {}

    def free(c):
        if c not in occupied:
            occupied[c] = (0 <= c[0] <= size and 0 <= c[1] <= size and
                           segment_clear(point(c), point(c), obstacles,
                                         clearance if c in (source, dest) else clearance+.02))
        return occupied[c]

    if not free(source) or not free(dest):
        return []
    queue = [(0., source)]
    costs, parents = {source: 0.}, {}
    visited = set()
    while queue and len(visited) < limit:
        _, here = heapq.heappop(queue)
        if here in visited:
            continue
        visited.add(here)
        if here == dest:
            route = [list(goal[:2])]
            while here != source:
                route.append(point(here))
                here = parents[here]
            route.reverse()
            # Keep the longest clear segments instead of stopping at every cell.
            simplified, anchor, index = [], start, 0
            while index < len(route):
                end = index
                while end+1 < len(route) and segment_clear(
                        anchor, route[end+1], obstacles, clearance):
                    end += 1
                if not segment_clear(anchor, route[end], obstacles, clearance):
                    return []
                simplified.append(list(route[end]))
                anchor, index = route[end], end+1
            return simplified
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            other = here[0]+dx, here[1]+dy
            if not free(other) or not segment_clear(
                    point(here), point(other), obstacles, clearance):
                continue
            cost = costs[here]+math.hypot(dx, dy)
            if cost < costs.get(other, math.inf):
                costs[other], parents[other] = cost, here
                h = math.dist(other, dest)
                heapq.heappush(queue, (cost+h, other))
    return []
