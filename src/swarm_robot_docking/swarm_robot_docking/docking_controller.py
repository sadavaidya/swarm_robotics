"""Local pose controller: approach a staging pose, align, then approach slowly."""

import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from swarm_robot_behaviors.control import clamp

from .protocol import decode, encode, fresh, LATCHED, now_seconds, number, run, setting, wrap
from .pose_provider import PoseProvider


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        self.robot = setting(self, 'robot_id', self.get_namespace().strip('/'))
        self.timeout = number(self, 'data_timeout', .7)
        self.max_v = number(self, 'max_linear_speed', .15)
        self.final_v = number(self, 'final_linear_speed', .035)
        self.max_w = number(self, 'max_angular_speed', .6)
        self.k_distance = number(self, 'k_distance', .8)
        self.k_heading = number(self, 'k_heading', 2.0)
        self.position_tolerance = number(self, 'position_tolerance', .004)
        self.yaw_tolerance = number(self, 'yaw_tolerance', .035)
        self.axle_offset = float(setting(self, 'axle_offset', .02))
        if (self.max_v > .20 or self.final_v > min(.06, self.max_v) or
                self.max_w > .8 or not math.isfinite(self.axle_offset)):
            raise ValueError('Invalid speed limits or axle offset')
        self.health = None
        self.assignments = None
        self.epoch = None
        self.generation = 0
        self.slot = None
        self.phase = 'AVAILABLE'
        self.last_state = None
        self.was_resetting = False
        self.poses = PoseProvider(self, [self.robot], 'world', self.timeout)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'swarm/local_state', 10)
        self.assignment_sub = self.create_subscription(
            String, '/swarm/assignments', self.on_assignments, LATCHED)
        self.health_sub = self.create_subscription(String, 'swarm/health', self.on_health, 10)
        self.timer = self.create_timer(1 / number(self, 'control_rate', 20.0), self.tick)

    def on_assignments(self, msg):
        value = decode(msg)
        if value and isinstance(value.get('members'), dict):
            self.assignments = value

    def on_health(self, msg):
        self.health = decode(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def axle_error(self, pose, target):
        # The inherited drive axle is 20 mm forward of base_link. Control its
        # ground projection so rotating to final yaw doesn't shift the target.
        x = pose.x + self.axle_offset * math.cos(pose.yaw)
        y = pose.y + self.axle_offset * math.sin(pose.yaw)
        tx = target[0] + self.axle_offset * math.cos(target[2])
        ty = target[1] + self.axle_offset * math.sin(target[2])
        dx, dy = tx-x, ty-y
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx)-pose.yaw)

    def drive_to(self, pose, target, speed):
        distance, alpha = self.axle_error(pose, target)
        beta = wrap(target[2] - pose.yaw - alpha)
        w = clamp(self.k_heading * alpha - .5 * beta, -self.max_w, self.max_w)
        v = min(speed, self.k_distance * distance) * max(0.0, math.cos(alpha))
        if abs(alpha) > .6:
            v = 0.0
        return v, w

    def tick(self):
        now = now_seconds(self)
        pose = self.poses.get(self.robot, now)
        health_ok = (self.health and self.health.get('enabled') and
                     fresh(self.health.get('stamp'), now, self.timeout))
        lease_ok = (self.assignments and
                    fresh(self.assignments.get('stamp'), now, self.timeout))
        member = self.assignments['members'].get(self.robot) if lease_ok else None
        v, w = 0.0, 0.0
        resetting = bool(lease_ok and self.assignments.get('resetting'))
        if resetting:
            if not self.was_resetting:
                self.poses.clear()
            self.epoch, self.slot = None, None
            self.phase = 'AVAILABLE'
            state = 'RESETTING'
        elif not health_ok or pose is None:
            state = 'FAILED' if self.health and not self.health.get('enabled') else 'IDLE'
        elif member is None:
            self.slot = None
            self.phase = 'AVAILABLE'
            state = 'AVAILABLE'
        else:
            epoch = self.assignments['epoch']
            if self.epoch != epoch or self.slot != member['id']:
                target = member['target']
                at_target = (math.hypot(pose.x - target[0], pose.y - target[1]) < .03 and
                             abs(wrap(target[2] - pose.yaw)) < self.yaw_tolerance)
                self.phase = 'WAITING_FOR_PEER' if at_target else 'APPROACHING'
            self.epoch, self.slot = epoch, member['id']
            self.generation = member['generation']
            if member.get('docked'):
                self.phase = 'DOCKED'
            elif (self.phase in ('FINAL_APPROACH', 'READY_TO_DOCK') and
                  not member['allow_final']):
                self.phase = 'ALIGNING'
            target = member['target']
            if self.phase == 'APPROACHING':
                distance, _ = self.axle_error(pose, member['staging'])
                if distance <= self.position_tolerance:
                    self.phase = 'ALIGNING'
                else:
                    v, w = self.drive_to(pose, member['staging'], self.max_v)
            elif self.phase == 'ALIGNING':
                error = wrap(target[2] - pose.yaw)
                if abs(error) <= self.yaw_tolerance:
                    self.phase = 'WAITING_FOR_PEER'
                else:
                    w = clamp(self.k_heading * error, -self.max_w, self.max_w)
            elif self.phase == 'WAITING_FOR_PEER':
                if member['allow_final']:
                    self.phase = 'FINAL_APPROACH'
            elif self.phase == 'FINAL_APPROACH':
                distance, _ = self.axle_error(pose, target)
                yaw_error = wrap(target[2] - pose.yaw)
                if distance > self.position_tolerance:
                    v, w = self.drive_to(pose, target, self.final_v)
                elif abs(yaw_error) > self.yaw_tolerance:
                    w = clamp(self.k_heading * yaw_error, -self.max_w, self.max_w)
                else:
                    self.phase = 'READY_TO_DOCK'
            state = self.phase
        self.was_resetting = resetting
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(encode({
            'stamp': now, 'robot_id': self.robot, 'state': state,
            'epoch': self.epoch, 'generation': self.generation, 'slot': self.slot,
        }))
        if state != self.last_state:
            self.get_logger().info(state + (' ' + self.slot if self.slot else ''))
            self.last_state = state


def main(args=None):
    run(DockingController, args)
