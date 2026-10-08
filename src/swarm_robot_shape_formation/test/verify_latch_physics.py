#!/usr/bin/env python3
"""Opt-in isolated-world test: consent, capture radius, real constraint and release.

Resets the fleet and teleports two stopped test robots to a clear fixture area.
Requires an explicit ROS_DOMAIN_ID; never run against a fleet you are using.
"""
import copy
import json
import math
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_robot_docking.protocol import decode, encode, LATCHED, wrap
from swarm_robot_shape_formation.peer_data import PEER_QOS


def main():
    if not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Use a dedicated test ROS_DOMAIN_ID')
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(Path(
        get_package_share_directory('swarm_robot_shape_formation'), 'config/fastdds_udp.xml')))
    rclpy.init()
    node = rclpy.create_node('verify_physical_latch')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    state, odom = {}, {}
    subs = [node.create_subscription(String, '/swarm/'+topic,
                                     lambda msg, t=topic: state.update({t: decode(msg)}), LATCHED)
            for topic in ('shape_goal', 'dock_status')]
    subs.append(node.create_subscription(
        String, '/robot_1/robot_description',
        lambda msg: state.update({'fixture_description': msg.data}), LATCHED))
    for name in [f'robot_{i}' for i in range(1, 11)]+['latch_fixture_z', 'latch_fixture_a']:
        subs.append(node.create_subscription(Odometry, '/'+name+'/odom',
                                             lambda msg, n=name: odom.update({n: msg}),
                                             qos_profile_sensor_data))
    publishers = {name: node.create_publisher(String, '/'+name+'/shape/dock_request', PEER_QOS)
                  for name in ('latch_fixture_z', 'latch_fixture_a')}

    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(predicate, timeout=30.):
        until = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < until, state
            rclpy.spin_once(node, timeout_sec=.02)

    def call(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=10.)
        future = client.call_async(request)
        wait(future.done, 90.)
        result = future.result()
        node.destroy_client(client)
        assert result.success, result
        spin(.5)

    def place(name, x, y):
        req = SetEntityState.Request()
        req.state.name, req.state.reference_frame = name, 'world'
        req.state.pose.position.x, req.state.pose.position.y = x, y
        req.state.pose.position.z, req.state.pose.orientation.w = .01, 1.
        call(SetEntityState, '/swarm/simulation/set_entity_state', req)

    def consent(names, duration=2., mismatch=False):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            for name in names:
                digest = 'physics-fixture'
                if mismatch and name == 'latch_fixture_z':
                    digest = 'wrong-task'
                publishers[name].publish(encode(dict(
                    epoch=state['shape_goal']['epoch'],
                    stamp=node.get_clock().now().nanoseconds/1e9,
                    robot=name, digest=digest,
                    connections=[dict(a='latch_fixture_z', b='latch_fixture_a')])))
            spin(.1)

    def relative():
        a, b = [odom[n].pose.pose for n in ('latch_fixture_z', 'latch_fixture_a')]

        def yaw(p):
            q = p.orientation
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        angle = yaw(a)
        dx, dy = b.position.x-a.position.x, b.position.y-a.position.y
        return [dx*math.cos(angle)+dy*math.sin(angle),
                -dx*math.sin(angle)+dy*math.cos(angle), wrap(yaw(b)-angle)]

    try:
        wait(lambda: len(odom) == 10 and all(state.values()) and len(state) == 3)
        call(Trigger, '/swarm/reset', Trigger.Request())
        others = [m.pose.pose.position for n, m in odom.items()
                  if n not in ('latch_fixture_z', 'latch_fixture_a')]
        x, y = next((float(x), float(y)) for x in range(-3, 3) for y in range(-3, 4)
                    if all(math.hypot(p.x-x, p.y-y) > 1. for p in others))
        for name, dx in [('latch_fixture_z', 0.), ('latch_fixture_a', .5)]:
            req = SpawnEntity.Request()
            req.name, req.robot_namespace = name, '/'+name
            req.xml = state['fixture_description'].replace('robot_1', name)
            req.initial_pose.position.x, req.initial_pose.position.y = x+dx, y
            req.initial_pose.position.z, req.initial_pose.orientation.w = .01, 1.
            call(SpawnEntity, '/spawn_entity', req)
        wait(lambda: all(name in odom for name in ('latch_fixture_z', 'latch_fixture_a')))
        spin(1.)
        consent(['latch_fixture_z', 'latch_fixture_a'])
        assert not state['dock_status']['bonds'], 'Distant robots must never latch'
        # Revoke the second robot's previous far-away consent BEFORE bringing
        # ports together. Wall time alone is insufficient when Gazebo runs slow.
        publishers['latch_fixture_a'].publish(encode(dict(
            epoch=state['shape_goal']['epoch'], stamp=node.get_clock().now().nanoseconds/1e9,
            robot='latch_fixture_a', digest='physics-fixture', connections=[])))
        wait(lambda: any(p.get('edge') == 'latch_fixture_z:latch_fixture_a' and
                         p.get('reason') == 'WAITING_FOR_BOTH_CONSENTS'
                         for p in state['dock_status'].get('pending', [])))
        place('latch_fixture_a', x+.22, y)
        spin(1.)
        consent(['latch_fixture_z'])
        assert not state['dock_status']['bonds'], 'Both neighbors must freshly consent'
        consent(['latch_fixture_z', 'latch_fixture_a'], mismatch=True)
        assert not state['dock_status']['bonds'], 'Different assignment digests must not latch'
        consent(['latch_fixture_z', 'latch_fixture_a'])
        wait(lambda: len(state['dock_status']['bonds']) == 1)
        before = relative()
        # Deliberately displace only one member; a status-only fake latch would
        # retain a 25 mm relative translation. A real constraint restores it.
        req = SetEntityState.Request()
        req.state.name, req.state.reference_frame = 'latch_fixture_a', 'world'
        req.state.pose = copy.deepcopy(odom['latch_fixture_a'].pose.pose)
        req.state.pose.position.y += .025
        call(SetEntityState, '/swarm/simulation/set_entity_state', req)
        spin(2.)
        after = relative()
        error = math.dist(before[:2], after[:2])
        assert error < .003 and abs(wrap(after[2]-before[2])) < .02, (before, after)
        print(json.dumps(dict(distant_capture_rejected=True, unilateral_capture_rejected=True,
                              assignment_mismatch_rejected=True,
                              physical_constraint_restores_relative_pose=True,
                              relative_position_error=error)), flush=True)
        call(Trigger, '/swarm/reset', Trigger.Request())
        wait(lambda: state['dock_status']['bonds'] == [])
        print('PASS: physical constraint, bilateral consent, capture radius and reset release',
              flush=True)
    finally:
        try:
            call(Trigger, '/swarm/detach_all', Trigger.Request())
            for name in ('latch_fixture_z', 'latch_fixture_a'):
                if name in odom:
                    call(DeleteEntity, '/delete_entity', DeleteEntity.Request(name=name))
        finally:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
