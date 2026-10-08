"""Discover partners, reserve disjoint pairs and negotiate peer-relative rendezvous."""

import math
import uuid

from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

from .auction import allocate
from .protocol import (
    decode, encode, finite_pose, fresh, LATCHED, now_seconds, number, run,
)
from .rendezvous import clear_pair_bids, rendezvous_offers
from .shape_library import docking_errors


class SwarmAllocator(Node):
    def __init__(self):
        super().__init__('swarm_allocator')
        self.timeout = number(self, 'heartbeat_timeout', 1.8)
        self.bid_wait = number(self, 'bid_collection_time', .6)
        self.dock_offset = number(self, 'dock_offset', .062)
        self.gap = number(self, 'dock_gap', .012)
        self.staging = number(self, 'staging_distance', .30)
        self.clearance = number(self, 'rendezvous_clearance', .7)
        self.separation = number(self, 'dock_separation_tolerance', .04)
        self.lateral = number(self, 'dock_lateral_tolerance', .02)
        self.yaw_tolerance = number(self, 'dock_yaw_tolerance', math.radians(5))
        self.dwell = number(self, 'dock_dwell', .5)
        if self.gap >= self.separation:
            raise ValueError('dock_gap must be below dock_separation_tolerance')
        self.robots = {}
        self.last_healthy = {}
        self.pairs = []
        self.goal = 0
        self.input_pair_count = 0
        self.epoch = uuid.uuid4().hex
        self.generation = 0
        self.pending = None
        self.resetting = False
        self.last_now = None
        self.last_active = None
        self.last_status = None
        self.notice = ''
        self.shape_pub = self.create_publisher(String, '/swarm/shape', LATCHED)
        self.assign_pub = self.create_publisher(String, '/swarm/assignments', LATCHED)
        self.registry_pub = self.create_publisher(String, '/swarm/registry', LATCHED)
        self.status_pub = self.create_publisher(String, '/swarm/docking_status', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/swarm/markers', LATCHED)
        self.command_sub = self.create_subscription(
            String, '/swarm/shape_command', self.command, 10)
        self.count_sub = self.create_subscription(
            Int32, '/swarm/pair_count', self.count_command, 10)
        self.robot_sub = self.create_subscription(String, '/swarm/robot_state', self.heartbeat, 50)
        self.hold_service = self.create_service(SetBool, '/swarm/hold', self.hold)
        self.timer = self.create_timer(.2, self.tick)
        self.publish_shape()

    def command(self, msg):
        name = msg.data.strip().upper()
        if name == 'PAIR':
            self.request(self.goal + 1)
        elif name == 'PAIR_2':
            self.request(2)
        else:
            self.get_logger().warning('Supported shape commands: PAIR and PAIR_2')

    def count_command(self, msg):
        self.request(msg.data)

    def request(self, count):
        if self.resetting:
            self.notice = 'RESET_IN_PROGRESS: retry after reset finishes'
        elif count < 1:
            self.notice = 'Pair count must be positive; use /swarm/reset to clear pairs'
        elif min(count, 2) < self.goal:
            self.notice = 'Use /swarm/reset before reducing the requested pair count'
        else:
            self.input_pair_count = count
            self.goal = min(count, 2)
            self.notice = (f'{count} pairs not possible with five robots; '
                           'forming the maximum of 2 pairs (four distinct robots)'
                           if count > 2 else '')
            self.get_logger().info(
                f'Received pair_count={count}; accepted {self.goal} total pair(s)')
            self.publish_shape()
        if self.notice:
            self.get_logger().warning(self.notice)
        self.tick()

    def hold(self, request, response):
        self.resetting = request.data
        if self.resetting:
            self.goal = 0
            self.input_pair_count = 0
            self.pairs.clear()
            self.pending = None
            self.robots.clear()
            self.last_healthy.clear()
            self.generation += 1
            self.epoch = uuid.uuid4().hex
            self.notice = ''
            self.publish_shape()
        self.tick()
        response.success = True
        response.message = 'All assignments cleared; motion held' if self.resetting else 'Ready'
        return response

    def heartbeat(self, msg):
        value = decode(msg)
        now = now_seconds(self)
        if (not value or not isinstance(value.get('robot_id'), str) or
                not value['robot_id'] or not fresh(value.get('stamp'), now, self.timeout)):
            return
        old = self.robots.get(value['robot_id'])
        if old is None or value['stamp'] >= old['stamp']:
            self.robots[value['robot_id']] = value
            if value.get('healthy', value.get('available')) and finite_pose(value.get('pose')):
                self.last_healthy[value['robot_id']] = value

    def active_states(self, now):
        active = {}
        for name, state in self.robots.items():
            if not fresh(state.get('stamp'), now, self.timeout):
                continue
            if state.get('healthy', state.get('available')) and finite_pose(state.get('pose')):
                active[name] = state
            elif fresh(self.last_healthy.get(name, {}).get('stamp'), now, self.timeout):
                # Reserve a recent member through brief pose/TF gaps. Its cached
                # pose is never used to authorize movement or declare docking.
                active[name] = dict(self.last_healthy[name], healthy=False,
                                    dock=None, state='WAITING_FOR_FRESH_POSE')
        return active

    def occupied(self):
        return {robot for pair in self.pairs for robot in pair['members'].values()}

    def publish_shape(self):
        slots = [slot for pair in self.pairs for slot in pair['slots']]
        if self.pending:
            slots += [slot for offer in self.pending['offers'] for slot in offer['slots']]
        self.shape_pub.publish(encode({
            'name': 'PAIR', 'epoch': self.epoch, 'requested_pairs': self.goal,
            'slots': slots, 'candidates': self.pending['offers'] if self.pending else [],
        }))

    def start_round(self, active, idle, needed, now):
        self.epoch = uuid.uuid4().hex
        self.generation += 1
        offers = rendezvous_offers(
            {name: active[name]['pose'] for name in idle}, self.dock_offset, self.gap,
            self.staging, [p['origin'] for p in self.pairs], self.clearance)
        self.pending = {'names': tuple(sorted(idle)), 'offers': offers,
                        'needed': needed, 'stamp': now}
        self.publish_shape()
        self.get_logger().info('Collecting bids for meeting poses derived from robot positions')

    def bids(self, active):
        return {name: state['bids'] for name, state in active.items()
                if state.get('healthy', state.get('available')) and
                state.get('shape_epoch') == self.epoch and isinstance(state.get('bids'), dict)}

    def repair(self, active):
        changed = False
        for pair in self.pairs:
            old = dict(pair['members'])
            pair['members'] = {slot: robot for slot, robot in old.items() if robot in active}
            if pair['members'] != old:
                for slot in set(old) - set(pair['members']):
                    self.get_logger().warning(f"{old[slot]} lost; reopening {slot}")
                pair['docked'] = pair['allow_final'] = False
                pair['close_since'] = None
            ids = [s['id'] for s in pair['slots']]
            if len(pair['members']) < 2:
                other_members = self.occupied() - set(pair['members'].values())
                table = {name: bids for name, bids in self.bids(active).items()
                         if name not in other_members and
                         all(isinstance(bids.get(slot), (int, float)) and
                             math.isfinite(bids[slot]) and bids[slot] >= 0 for slot in ids)}
                new = allocate(ids, table, pair['members'])
                if new:
                    pair['members'] = new
            if pair['members'] != old:
                self.generation += 1
                pair['generation'] = self.generation
                changed = True
                for slot, robot in pair['members'].items():
                    if old.get(slot) != robot:
                        self.get_logger().info(f'{robot} -> {slot}')
        if changed:
            self.publish_shape()

    def plan(self, active, now):
        if len(self.pairs) >= self.goal or any(not p['docked'] for p in self.pairs):
            return
        idle = set(active) - self.occupied()
        needed = self.goal - len(self.pairs)
        if len(idle) < 2 * needed:
            return
        if self.pending is None:
            self.start_round(active, idle, needed, now)
        if now - self.pending['stamp'] < self.bid_wait:
            return
        selected = clear_pair_bids(
            self.pending['offers'], self.bids(active), needed, self.clearance)
        if not selected:
            # A brief health/TF gap must not repeatedly invalidate everybody's
            # bids. Refresh only when the old offer set cannot make progress.
            if (now-self.pending['stamp'] > max(2., 3*self.bid_wait) and
                    self.pending['names'] != tuple(sorted(idle))):
                self.start_round(active, idle, needed, now)
            return
        for offer in selected:
            pair_id = f'pair_{len(self.pairs)+1}'
            slots, members = [], {}
            for slot in offer['slots']:
                new_slot = dict(slot, id=pair_id + '/' + slot['id'].split('/')[-1])
                slots.append(new_slot)
                members[new_slot['id']] = offer['owners'][slot['id']]
            self.generation += 1
            self.pairs.append({
                'id': pair_id, 'origin': offer['origin'], 'slots': slots, 'members': members,
                'generation': self.generation, 'allow_final': False, 'docked': False,
                'close_since': None, 'errors': None,
            })
            self.get_logger().info(f'{pair_id}: {members}; rendezvous {offer["origin"]}')
        self.pending = None
        self.publish_shape()

    def progress(self, pair, active, now):
        if len(pair['members']) != 2:
            pair['status'] = 'WAITING_FOR_HEALTHY_PARTNER'
            return
        states = [active[pair['members'][s['id']]] for s in pair['slots']]
        if any(not s.get('healthy', s.get('available')) for s in states):
            pair['status'] = 'DOCKED' if pair['docked'] else 'WAITING_FOR_FRESH_POSE'
            if not pair['docked']:
                pair['allow_final'] = False
                pair['close_since'] = None
            return
        synchronized = all(s.get('assignment_epoch') == self.epoch and
                           s.get('generation') == pair['generation'] for s in states)
        ready = ('WAITING_FOR_PEER', 'FINAL_APPROACH', 'READY_TO_DOCK', 'DOCKED')
        if synchronized and all(s.get('state') in ready for s in states):
            pair['allow_final'] = True
        pair['status'] = 'ASSEMBLING'
        if (synchronized and pair['allow_final'] and
                all(s.get('state') in ready[1:] and finite_pose(s.get('dock')) for s in states)):
            errors = docking_errors(states[0]['dock'], states[1]['dock'])
            pair['errors'] = errors
            close = (errors['separation'] < self.separation and
                     errors['lateral'] < self.lateral and errors['yaw_error'] < self.yaw_tolerance
                     and all(math.dist(s['pose'][:2], slot['target'][:2]) < .03
                             for s, slot in zip(states, pair['slots'])))
            if not close:
                pair['close_since'] = None
            elif pair['close_since'] is None:
                pair['close_since'] = now
            if pair['close_since'] is not None and now - pair['close_since'] >= self.dwell:
                if not pair['docked']:
                    self.get_logger().info(pair['id'] + ' DOCKED: ' + str(pair['members']))
                pair['docked'] = True
        if pair['docked']:
            pair['status'] = 'DOCKED'

    def tick(self):
        now = now_seconds(self)
        if self.last_now is not None and now < self.last_now:
            self.get_logger().warning(
                'Simulation clock moved backwards; clearing old assignments and recollecting bids')
            self.robots.clear()
            self.last_healthy.clear()
            self.pairs.clear()
            self.pending = None
            self.epoch = uuid.uuid4().hex
            self.generation += 1
            self.publish_shape()
        self.last_now = now
        active = self.active_states(now)
        eligible = {n: s for n, s in active.items() if s.get('healthy', s.get('available'))}
        if sorted(active) != self.last_active:
            self.get_logger().info('Healthy robots: ' + ' '.join(sorted(active)))
            self.last_active = sorted(active)
        if not self.resetting and self.goal:
            self.repair(active)
            self.plan(eligible, now)
            for pair in self.pairs:
                self.progress(pair, active, now)
        occupied = self.occupied()
        registry = {name: dict(value, healthy=name in eligible, occupied=name in occupied,
                               available=name in eligible and name not in occupied,
                               state=value.get('state') if name in eligible else 'FAILED',
                               alive=fresh(value.get('stamp'), now, self.timeout))
                    for name, value in self.robots.items()}
        self.registry_pub.publish(encode({'stamp': now, 'robots': registry}))
        completed = sum(p['docked'] for p in self.pairs)
        complete = self.goal > 0 and completed == self.goal
        if self.resetting:
            status = 'RESETTING'
        elif not self.goal:
            status = 'WAITING_FOR_REQUEST'
        elif complete:
            status = 'DOCKED'
        elif len(eligible) < 2 * self.goal:
            status = 'INSUFFICIENT_HEALTHY_ROBOTS'
        elif self.pending:
            status = ('COLLECTING_BIDS' if now-self.pending['stamp'] < self.bid_wait
                      else 'WAITING_FOR_RENDEZVOUS_BIDS')
        else:
            status = 'ASSEMBLING'
        members = {}
        for pair in self.pairs:
            for slot in pair['slots']:
                robot = pair['members'].get(slot['id'])
                if robot:
                    members[robot] = dict(slot, pair_id=pair['id'],
                                          generation=pair['generation'],
                                          allow_final=pair['allow_final'], docked=pair['docked'])
        envelope = {'stamp': now, 'epoch': self.epoch, 'generation': self.generation,
                    'members': members, 'resetting': self.resetting,
                    'allow_final': complete, 'docked': complete}
        self.assign_pub.publish(encode(envelope))
        pairs = [{k: v for k, v in p.items() if k != 'close_since'} for p in self.pairs]
        self.status_pub.publish(encode(dict(
            envelope, status=status, requested_pairs=self.goal,
            input_pair_count=self.input_pair_count, completed_pairs=completed,
            complete=complete, pairs=pairs, notice=self.notice,
            errors=pairs[0]['errors'] if len(pairs) == 1 else None)))
        if status != self.last_status:
            self.get_logger().info(f'{status}: {completed}/{self.goal} pairs')
            self.last_status = status
        self.markers()

    def markers(self):
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)
        for pair_index, pair in enumerate(self.pairs):
            for index, slot in enumerate(pair['slots']):
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id = pair['id'], index
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.position.x, marker.pose.position.y = slot['target'][:2]
                marker.pose.position.z = .20 + .10*index
                marker.pose.orientation.w = 1.0
                marker.scale.z = .06
                marker.color.r, marker.color.g, marker.color.b = 1.0, .8, float(pair_index)
                marker.color.a = 1.0
                marker.text = slot['id'] + ': ' + pair['members'].get(slot['id'], 'waiting')
                array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    run(SwarmAllocator, args)
