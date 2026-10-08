"""ROS-independent planar control and narrow front-sensor safety."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """A planar pose already expressed in the common world frame."""

    x: float
    y: float
    yaw: float


def clamp(value, low, high):
    return max(low, min(high, value))


def relative_target(own, target):
    dx, dy = target.x - own.x, target.y - own.y
    bearing = math.atan2(dy, dx) - own.yaw
    return math.hypot(dx, dy), math.atan2(math.sin(bearing), math.cos(bearing))


def proportional_command(distance, bearing, spacing, k_distance, k_heading,
                         max_linear, max_angular, distance_deadband, heading_deadband,
                         turn_in_place_angle):
    """Turn first when badly misaligned; approach without blindly reversing."""
    error = distance - spacing
    linear = 0.0 if error <= distance_deadband else min(max_linear, k_distance * error)
    linear *= max(0.0, math.cos(bearing))
    if abs(bearing) >= turn_in_place_angle:
        linear = 0.0
    angular = (0.0 if abs(bearing) <= heading_deadband else
               clamp(k_heading * bearing, -max_angular, max_angular))
    return linear, angular


def scan_minimum(ranges):
    """+inf is clear, -inf is too close, all-invalid scans are unavailable."""
    valid = [0.0 if value == -math.inf else value
             for value in ranges if not math.isnan(value) and (value >= 0 or value == -math.inf)]
    return min(valid) if valid else None


class RangeGuard:
    """Latched stop-and-left-turn override with clearance hysteresis."""

    def __init__(self, stop_distance, clear_distance, turn_speed, min_turn_time):
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.turn_speed = turn_speed
        self.min_turn_time = min_turn_time
        self.started = None

    def reset(self):
        self.started = None

    def command(self, distance, now):
        if self.started is not None and now < self.started:
            self.reset()
        if self.started is None and distance < self.stop_distance:
            self.started = now
        if self.started is not None:
            if distance >= self.clear_distance and now - self.started >= self.min_turn_time:
                self.reset()
            else:
                return 0.0, self.turn_speed
        return None
