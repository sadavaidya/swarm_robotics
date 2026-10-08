"""Single-assignment bid/max-consensus auction; no central clearing node."""
import hashlib
import json
import math


def better(a, b):
    if a['owner'] is None:
        return False
    if b['owner'] is None:
        return True
    return a['bid'] > b['bid'] or (a['bid'] == b['bid'] and a['owner'] < b['owner'])


class Auction:
    def __init__(self, robot, robots, scores):
        self.robot, self.robots, self.scores = robot, set(robots), scores
        self.winners = [dict(owner=None, bid=0.) for _ in scores]

    def merge(self, incoming):
        if not isinstance(incoming, list) or len(incoming) != len(self.winners):
            return
        for i, candidate in enumerate(incoming):
            if (not isinstance(candidate, dict) or candidate.get('owner') not in self.robots or
                    not isinstance(candidate.get('bid'), (int, float)) or
                    not math.isfinite(candidate['bid'])):
                continue
            if better(candidate, self.winners[i]):
                self.winners[i] = dict(candidate)

    def bid(self):
        # A robot keeps its winning role until a stronger peer claim displaces it.
        if any(w['owner'] == self.robot for w in self.winners):
            return
        candidates = [(score, -i, i) for i, score in enumerate(self.scores)
                      if better(dict(owner=self.robot, bid=score), self.winners[i])]
        if candidates:
            _, _, i = max(candidates)
            self.winners[i] = dict(owner=self.robot, bid=self.scores[i])

    @property
    def complete(self):
        owners = [w['owner'] for w in self.winners]
        return None not in owners and len(set(owners)) == len(owners)

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(self.winners, sort_keys=True).encode()).hexdigest()[:16]

    @property
    def role(self):
        return next((i for i, w in enumerate(self.winners) if w['owner'] == self.robot), None)
