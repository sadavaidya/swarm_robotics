"""Slow looping waypoint leader with the same front-range emergency guard."""

import math

from .control import Pose2D, proportional_command, relative_target
from .controller_base import ControllerBase, run


class LeaderController(ControllerBase):
    """Drive a rectangle well inside the arena using privileged world pose."""

    def __init__(self):
        super().__init__('leader_controller', 'robot_1')
        self.speed = min(self.max_linear, self.positive('cruise_speed', 0.12))
        self.tolerance = self.positive('waypoint_tolerance', 0.20)
        self.startup_delay = self.positive('startup_delay', 3.0)
        flat = self.setting('waypoints', [2.5, 0.0, 2.5, 2.5, -2.0, 2.5, -2.0, 0.0])
        if len(flat) < 4 or len(flat) % 2 or not all(math.isfinite(v) for v in flat):
            raise ValueError('waypoints must contain at least two finite x,y pairs')
        self.waypoints = [Pose2D(flat[i], flat[i+1], 0.0) for i in range(0, len(flat), 2)]
        self.index = 0
        self.ready_since = None
        self.start([self.robot])

    def nominal_command(self, own, now):
        if self.ready_since is None or now < self.ready_since:
            self.ready_since = now
        if now - self.ready_since < self.startup_delay:
            return (0.0, 0.0), 'startup_delay'
        distance, bearing = relative_target(own, self.waypoints[self.index])
        if distance < self.tolerance:
            self.index = (self.index + 1) % len(self.waypoints)
            distance, bearing = relative_target(own, self.waypoints[self.index])
        command = proportional_command(
            distance, bearing, 0.0, self.k_distance, self.k_heading,
            self.speed, self.max_angular, self.distance_deadband,
            self.heading_deadband, self.turn_angle)
        return command, 'waypoint_' + str(self.index)


def main(args=None):
    run(LeaderController, args)
