"""Identical peer auction, motion negotiation and local control on each robot."""
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from swarm_robot_docking.docking_controller import DockingController
from swarm_robot_docking.pose_provider import PoseProvider
from swarm_robot_docking.protocol import (
    decode, encode, LATCHED, now_seconds, run, setting, wrap,
)

from .auction import Auction
from .planning import path_to, segment_clear
from .concurrent import safe_command
from .consent import connections_for
from .static_frames import StaticFrames


from .peer_data import fresh_peer, PEER_QOS


class ShapeAgent(Node):
    def __init__(self):
        super().__init__('shape_agent')
        self.robot = setting(self, 'robot_id', self.get_namespace().strip('/'))
        self.names = list(setting(self, 'robot_ids', [self.robot]))
        self.timeout = float(setting(self, 'data_timeout', 1.2))
        self.peer_timeout = float(setting(self, 'peer_timeout', 1.8))
        self.clearance = float(setting(self, 'clearance', .30))
        self.max_v, self.max_w = .15, .6
        self.k_distance, self.k_heading, self.axle_offset = .8, 2., .02
        self.position_tolerance, self.yaw_tolerance = .006, .025
        self.staged, self.assembling = False, False
        self.reconfiguring = False
        self.docks = {}
        self.enabled, self.goal, self.auction = True, None, None
        self.epoch, self.round = None, 0
        self.peers = {}
        self.last_now, self.stable_since, self.digest = None, None, None
        self.route, self.route_key, self.replan_at = [], None, 0.
        self.last_publish, self.last_phase = -1., None
        self.static_frames = StaticFrames(self, self.robot)
        self.poses = PoseProvider(self, [self.robot], 'world', self.timeout)
        self.cmd_pub = self.create_publisher(Twist, 'shape/cmd_vel', 10)
        self.peer_pub = self.create_publisher(String, 'shape/peer_state', PEER_QOS)
        self.dock_pub = self.create_publisher(String, 'shape/dock_request', PEER_QOS)
        self.dock_sub = self.create_subscription(
            String, '/swarm/dock_status', self.on_docks, LATCHED)
        self.state_pub = self.create_publisher(String, 'swarm/local_state', 10)
        self.goal_sub = self.create_subscription(
            String, '/swarm/shape_goal', self.on_goal, LATCHED)
        self.peer_subs = [self.create_subscription(
            String, '/'+name+'/shape/peer_state', self.on_peer, PEER_QOS)
            for name in self.names if name != self.robot]
        self.available_service = self.create_service(SetBool, 'set_available', self.available)
        self.reset_service = self.create_service(Trigger, 'swarm/reset', self.reset)
        self.timer = self.create_timer(.05, self.tick)

    def on_docks(self, msg):
        value = decode(msg)
        if value:
            self.docks = value

    def clear_task(self):
        self.staged, self.assembling = False, False
        self.reconfiguring = False
        self.auction, self.digest, self.stable_since = None, None, None
        self.route, self.route_key = [], None
        self.peers.clear()
        self.stop()

    def on_goal(self, msg):
        value = decode(msg)
        if not value or not isinstance(value.get('slots'), list) or not value.get('epoch'):
            return
        if value['epoch'] != self.epoch:
            self.clear_task()
            self.epoch, self.round = value['epoch'], 0
            self.reconfiguring = bool(value.get('reconfigure'))
        self.goal = value

    def on_peer(self, msg):
        value = decode(msg)
        if (not value or value.get('robot') not in self.names or
                value['robot'] == self.robot or value.get('epoch') != self.epoch or
                not isinstance(value.get('stamp'), (int, float)) or
                not math.isfinite(value['stamp'])):
            return
        # Cache before checking clock freshness: a receiver recovering from a
        # pause may consume peer samples before its queued /clock update. Tick
        # applies the freshness gate before bidding or authorizing movement.
        if value.get('round', 0) > self.round:
            self.clear_task()
            self.round = value['round']
        if value.get('round') == self.round:
            self.peers[value['robot']] = value

    def available(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.stop()
        response.success = True
        response.message = 'Enabled' if self.enabled else 'Disabled; stopped'
        return response

    def reset(self, request, response):
        self.enabled = True
        self.poses.clear()
        self.clear_task()
        response.success, response.message = True, 'Agent reset'
        return response

    def stop(self):
        self.cmd_pub.publish(Twist())

    def at_target(self, pose, target):
        return (math.dist(pose[:2], target[:2]) < self.position_tolerance and
                abs(wrap(pose[2]-target[2])) < self.yaw_tolerance)

    axle_error = DockingController.axle_error

    def navigation(self, pose, target, obstacles, now):
        clearance = .142 if self.assembling or self.reconfiguring else self.clearance
        key = tuple(target)
        if self.route_key != key:
            self.route, self.route_key = [], key
            self.replan_at = 0.
        blocked = not self.route or not segment_clear(
            [pose.x, pose.y], self.route[0], obstacles, clearance)
        if blocked and now >= self.replan_at:
            margin = clearance if self.assembling or self.reconfiguring else clearance+.05
            self.route = path_to([pose.x, pose.y], target, obstacles, margin)
            self.replan_at = now+.5
        if not self.route or not segment_clear(
                [pose.x, pose.y], self.route[0], obstacles, clearance):
            # A moving peer can enter our planned clearance bubble. Back out
            # before A*: its first grid cell otherwise stays permanently occupied.
            dx = dy = 0.
            for ox, oy in obstacles:
                distance = math.hypot(pose.x-ox, pose.y-oy)
                if 1e-6 < distance < clearance+.12:
                    weight = (clearance+.12-distance)/distance
                    dx += (pose.x-ox)*weight
                    dy += (pose.y-oy)*weight
            if math.hypot(dx, dy) > .001:
                bearing = wrap(math.atan2(dy, dx)-pose.yaw)
                direction = 1.
                if abs(bearing) > math.pi/2:
                    bearing = wrap(bearing+math.pi)
                    direction = -1.
                return (.04*direction if abs(bearing) < .5 else 0.,
                        max(-self.max_w, min(self.max_w, 2*bearing)), 'CLEARING')
            return 0., 0., 'BLOCKED'
        while len(self.route) > 1 and math.dist([pose.x, pose.y], self.route[0]) < .02:
            self.route.pop(0)
        final = len(self.route) == 1
        heading = target[2] if final else math.atan2(
            self.route[1][1]-self.route[0][1], self.route[1][0]-self.route[0][0])
        waypoint = list(self.route[0])+[heading]
        distance, bearing = self.axle_error(pose, waypoint)
        if final and distance < .004:
            error = wrap(target[2]-pose.yaw)
            return 0., max(-self.max_w, min(self.max_w, self.k_heading*error)), 'ALIGNING'
        # Follow the clear segment first; rotate to final yaw at the target.
        # Coupling the next heading into transit can cut an obstacle corner.
        v = min(self.max_v, self.k_distance*distance)*max(0., math.cos(bearing))
        if abs(bearing) > .6:
            v = 0.
        w = max(-self.max_w, min(self.max_w, self.k_heading*bearing))
        return v, w, 'MOVING'

    def tick(self):
        now = now_seconds(self)
        if self.last_now is not None and now < self.last_now:
            self.round += 1
            self.clear_task()
        self.last_now = now
        pose = self.poses.get(self.robot, now)
        xyz = [pose.x, pose.y, pose.yaw] if pose else None
        healthy = bool(self.enabled and pose)
        slots = self.goal['slots'] if self.goal else []
        phase, ready, arrived, proposal = 'AVAILABLE', False, False, None
        v, w = 0., 0.
        role, consents = None, []
        peers = {n: p for n, p in self.peers.items()
                 if fresh_peer(p.get('stamp'), now, self.peer_timeout) and p.get('healthy')}
        if self.goal and self.goal.get('resetting'):
            phase = 'RESETTING'
        elif self.goal and self.goal.get('transition'):
            phase = 'TRANSITION_HOLD'
        elif not healthy:
            phase = 'DISABLED' if not self.enabled else 'WAITING_FOR_POSE'
        elif slots:
            if self.auction is None:
                scores = [1000.-math.dist(xyz[:2], s['staging'][:2]) -
                          .08*abs(wrap(xyz[2]-s['target'][2])) for s in slots]
                self.auction = Auction(self.robot, self.names, scores)
            for peer in peers.values():
                self.auction.merge(peer.get('winners'))
            self.auction.bid()
            digest, role = self.auction.digest, self.auction.role
            if digest != self.digest:
                self.digest, self.stable_since = digest, now
                self.route_key = None
                self.staged, self.assembling = False, False
            agreed = (self.auction.complete and len(peers) == len(self.names)-1 and
                      all(p.get('digest') == digest for p in peers.values()))
            stable_since = self.stable_since if self.stable_since is not None else now
            ready = bool(agreed and now-stable_since >= .8)
            arrived = role is not None and self.at_target(xyz, slots[role]['target'])
            phase = 'AUCTION' if not agreed else 'CONSENSUS'
            if ready and all(p.get('ready') for p in peers.values()):
                if self.reconfiguring and all(
                        math.dist(xyz[:2], p['pose'][:2]) > .42 for p in peers.values()):
                    self.reconfiguring = False
                owners = [win['owner'] for win in self.auction.winners]
                target = slots[role]['target']
                staging = slots[role]['staging']
                if not self.staged and math.dist(xyz[:2], staging[:2]) < .035:
                    self.staged = True
                if self.staged and all(p.get('staged') for p in peers.values()):
                    self.assembling = True
                dock_fresh = (self.docks.get('epoch') == self.epoch and
                              fresh_peer(self.docks.get('stamp'), now, 1.))
                bonds = self.docks.get('bonds', []) if dock_fresh else []
                attached = any(self.robot in (b['a'], b['b']) for b in bonds)
                arrived = self.at_target(xyz, target) or attached
                phase = 'DOCKED' if attached else 'WAITING_FOR_DOCK'
                if not dock_fresh:
                    phase = 'WAITING_FOR_DOCK_PHYSICS'
                elif attached:
                    self.staged, self.assembling = True, True
                elif not arrived or not self.assembling:
                    if self.staged and not self.assembling:
                        phase = 'STAGED'
                    else:
                        destination = target if self.assembling else staging
                        obstacles = [p['pose'][:2] for p in peers.values()]
                        v, w, phase = self.navigation(pose, destination, obstacles, now)
                        v, w = safe_command(
                            self.robot, xyz, v, w, peers, self.assembling or self.reconfiguring)
                        if self.assembling or self.reconfiguring:
                            v = max(-.06, min(.06, v))
                        if phase == 'MOVING' and abs(v)+abs(w) < 1e-6:
                            phase = 'YIELDING'
                            self.route = []
                if dock_fresh and self.assembling and arrived:
                    consents = connections_for(
                        role, owners, self.goal['shape'] != 'LINE', peers, bonds)
            elif len(peers) != len(self.names)-1:
                phase = 'WAITING_FOR_PEERS'
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)
        local = dict(stamp=now, robot_id=self.robot, state=phase, epoch=self.epoch,
                     generation=self.round, slot=slots[role]['id'] if role is not None else None)
        self.state_pub.publish(encode(local))
        if phase != self.last_phase:
            self.get_logger().info(phase)
            self.last_phase = phase
        if now-self.last_publish >= .2 or now < self.last_publish:
            self.last_publish = now
            self.dock_pub.publish(encode(dict(
                epoch=self.epoch, digest=self.digest, stamp=now,
                robot=self.robot, connections=consents)))
            self.peer_pub.publish(encode(dict(
                stamp=now, robot=self.robot, epoch=self.epoch, round=self.round,
                peer_count=len(peers),
                missing_peers=sorted(set(self.names)-{self.robot}-set(peers)),
                pose=xyz, healthy=healthy, state=phase, ready=ready, arrived=arrived,
                role=role, proposal=proposal, digest=self.digest,
                staged=self.staged, assembling=self.assembling, command=[v, w],
                winners=self.auction.winners if self.auction else [])))


def main(args=None):
    run(ShapeAgent, args)
