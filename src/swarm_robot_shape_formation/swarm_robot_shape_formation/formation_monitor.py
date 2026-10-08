"""Read-only progress/visualization observer; agents never consume its outputs."""
import math

from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from swarm_robot_docking.protocol import (
    decode, encode, LATCHED, now_seconds, run, setting, wrap,
)


from .shapes import edges, port
from .peer_data import fresh_peer, PEER_QOS


class FormationMonitor(Node):
    def __init__(self):
        super().__init__('formation_monitor')
        self.names = list(setting(self, 'robot_ids', ['robot_1']))
        self.goal, self.peers, self.last_now, self.close_since = None, {}, None, None
        self.docks = {}
        self.dock_sub = self.create_subscription(
            String, '/swarm/dock_status', self.on_docks, LATCHED)
        self.status_pub = self.create_publisher(String, '/swarm/formation_status', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/swarm/shape_markers', LATCHED)
        self.goal_sub = self.create_subscription(
            String, '/swarm/shape_goal', self.on_goal, LATCHED)
        self.peer_subs = [self.create_subscription(
            String, '/'+name+'/shape/peer_state', self.on_peer, PEER_QOS)
            for name in self.names]
        self.timer = self.create_timer(.2, self.tick)

    def on_docks(self, msg):
        value = decode(msg)
        if value:
            self.docks = value

    def on_goal(self, msg):
        value = decode(msg)
        if not value:
            return
        if value.get('epoch') != (self.goal or {}).get('epoch'):
            self.close_since = None
        self.goal = value

    def on_peer(self, msg):
        value = decode(msg)
        if value and value.get('robot') in self.names:
            self.peers[value['robot']] = value

    def tick(self):
        now = now_seconds(self)
        if self.last_now is not None and now < self.last_now:
            self.peers.clear()
            self.close_since = None
        self.last_now = now
        goal = self.goal or {}
        peers = {n: p for n, p in self.peers.items() if p.get('epoch') == goal.get('epoch')
                 and fresh_peer(p.get('stamp'), now, 1.8)}
        slots = goal.get('slots', [])
        members, errors = {}, {}
        ready = (len(peers) == len(self.names) and all(p.get('healthy') for p in peers.values())
                 and len({p.get('digest') for p in peers.values()}) == 1
                 and len({p.get('round') for p in peers.values()}) == 1
                 and all(p.get('ready') for p in peers.values()))
        if ready and slots:
            winners = next(iter(peers.values()))['winners']
            for slot, winner in zip(slots, winners):
                owner = winner['owner']
                if owner not in peers:
                    continue
                pose = peers[owner]['pose']
                target = slot['target']
                members[owner] = slot
                errors[owner] = dict(position=math.dist(pose[:2], target[:2]),
                                     yaw=abs(wrap(pose[2]-target[2])))
        bonds = self.docks.get('bonds', [])
        actual = {(b['a'], b['b']) for b in bonds}
        expected = set()
        if ready and slots and len(members) == len(slots):
            owners = [w['owner'] for w in winners]
            expected = {(owners[a], owners[b]) for a, b in edges(
                len(slots), goal.get('shape') != 'LINE')}
        connected = (bool(expected) and actual == expected and
                     self.docks.get('epoch') == goal.get('epoch') and
                     self.docks.get('physical') is True and
                     fresh_peer(self.docks.get('stamp'), now, 1.))
        close = (connected and bool(slots) and len(members) == len(slots) and
                 all(e['position'] < .05 and e['yaw'] < .25 for e in errors.values()))
        if not close:
            self.close_since = None
        elif self.close_since is None:
            self.close_since = now
        complete = self.close_since is not None and now-self.close_since >= .5
        status = 'WAITING_FOR_REQUEST'
        if goal.get('resetting'):
            status = 'RESETTING'
        elif goal.get('transition'):
            stage = goal['transition']['stage']
            status = {'HOLD': 'STOPPING_FOR_SHAPE_CHANGE', 'RELEASE': 'DISCONNECTING',
                      'FAILED': 'TRANSITION_FAILED'}.get(stage, 'TRANSITION')
        elif slots:
            if complete:
                status = 'COMPLETE'
            elif len(peers) < len(self.names) or not all(p.get('healthy') for p in peers.values()):
                status = 'WAITING_FOR_PEERS'
            elif not ready:
                status = 'AUCTION'
            else:
                blocked = any(p['state'] == 'BLOCKED' for p in peers.values())
                status = 'BLOCKED' if blocked else 'FORMING'
        self.status_pub.publish(encode(dict(
            stamp=now, epoch=goal.get('epoch'), shape=goal.get('shape'), status=status,
            complete=complete, members=members, errors=errors, robots=peers,
            requested_shape=goal.get('transition', {}).get('requested_shape'),
            mechanical_attachment=connected, bonds=bonds,
            expected_bonds=len(expected), connected_bonds=len(actual),
            moving_robots=[n for n, p in peers.items()
                           if sum(abs(v) for v in p.get('command', [])) > .01])))
        self.markers(slots, members, complete)

    def markers(self, slots, members, complete):
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)
        outline = Marker()
        outline.header.frame_id, outline.ns, outline.id = 'world', 'outline', 0
        outline.type = Marker.LINE_STRIP
        outline.pose.orientation.w = 1.
        outline.scale.x = .015
        outline.color.r, outline.color.g, outline.color.a = 1., .8, .8
        for i, slot in enumerate(slots):
            marker = Marker()
            marker.header.frame_id, marker.ns, marker.id = 'world', 'roles', i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose.position.x, marker.pose.position.y = slot['target'][:2]
            marker.pose.position.z, marker.pose.orientation.w = .22, 1.
            marker.scale.z = .065
            marker.color.g, marker.color.r, marker.color.a = 1., float(not complete), 1.
            owner = next((n for n, s in members.items() if s['id'] == slot['id']), 'auction')
            marker.text = slot['id'] + ': ' + owner
            array.markers.append(marker)
            outline.points.append(Point(x=slot['target'][0], y=slot['target'][1], z=.02))
        if slots and self.goal['shape'] != 'LINE':
            outline.points.append(outline.points[0])
        array.markers.append(outline)
        # Green spheres are measured physical latches, not target markers.
        for i, bond in enumerate(self.docks.get('bonds', [])):
            a, b = self.peers.get(bond['a'], {}), self.peers.get(bond['b'], {})
            if not a.get('pose') or not b.get('pose'):
                continue
            front, rear = port(a['pose']), port(b['pose'], False)
            marker = Marker()
            marker.header.frame_id, marker.ns, marker.id = 'world', 'physical_latches', i
            marker.type = Marker.SPHERE
            marker.pose.orientation.w = 1.
            marker.pose.position.x = (front[0]+rear[0])/2
            marker.pose.position.y = (front[1]+rear[1])/2
            marker.pose.position.z = .08
            marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.g, marker.color.a = 1., 1.
            array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    run(FormationMonitor, args)
