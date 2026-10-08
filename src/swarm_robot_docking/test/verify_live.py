#!/usr/bin/env python3
"""Opt-in integration verification; resets a dedicated five-robot simulation."""
import argparse
import json
import math
import time

from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, SetBool

from swarm_robot_docking.protocol import decode, LATCHED


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--failure-only', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_relative_pairs')
    state, odom, cmd = {}, {}, {}
    subscriptions = []
    for key in ('registry', 'assignments', 'docking_status'):
        subscriptions.append(node.create_subscription(
            String, '/swarm/' + key,
            lambda m, k=key: state.update({k: decode(m)}), LATCHED))
    for i in range(1, 6):
        name = f'robot_{i}'
        subscriptions.extend([
            node.create_subscription(Odometry, '/'+name+'/odom',
                                     lambda m, n=name: odom.update({n: m}),
                                     qos_profile_sensor_data),
            node.create_subscription(Twist, '/'+name+'/cmd_vel',
                                     lambda m, n=name: cmd.update({n: m}), 10)])
    count = node.create_publisher(Int32, '/swarm/pair_count', 10)
    shape = node.create_publisher(String, '/swarm/shape_command', 10)

    def status():
        return state.get('docking_status', {})

    def wait(predicate, seconds=40):
        deadline = time.monotonic() + seconds
        while not predicate():
            assert time.monotonic() < deadline, json.dumps(status())
            rclpy.spin_once(node, timeout_sec=.02)

    def settle(seconds=1.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def service(kind, topic, request):
        client = node.create_client(kind, topic)
        assert client.wait_for_service(timeout_sec=10), topic
        future = client.call_async(request)
        wait(future.done, 60)
        result = future.result()
        assert result.success, str(result)
        node.destroy_client(client)
        settle(.5)
        return result

    def reset():
        service(Trigger, '/swarm/reset', Trigger.Request())
        wait(lambda: status().get('requested_pairs') == 0 and
             len(state.get('registry', {}).get('robots', {})) == 5 and
             all(s['healthy'] and s['state'] == 'AVAILABLE'
                 for s in state['registry']['robots'].values()))
        stopped()

    def xy(name):
        p = odom[name].pose.pose.position
        return [p.x, p.y]

    def stopped():
        settle(.5)
        assert len(cmd) == 5
        assert all(c.linear.x == c.angular.z == 0 for c in cmd.values())

    def docked(n):
        wait(lambda: status().get('complete') and status().get('completed_pairs') == n, 150)
        settle(1.)
        assert len(status()['members']) == n*2
        for pair in status()['pairs']:
            assert pair['docked']
            assert pair['errors']['separation'] < .04
            assert pair['errors']['lateral'] < .02
            assert pair['errors']['yaw_error'] < math.radians(5)
        stopped()

    report = {}
    try:
        wait(lambda: len(odom) == 5 and count.get_subscription_count() > 0)
        reset()
        initial = {n: xy(n) for n in odom}
        if args.failure_only:
            count.publish(Int32(data=1))
            wait(lambda: len(status().get('members', {})) == 2)
            first = dict(status()['members'])
            failed = sorted(first)[-1]
            survivor = next(n for n in first if n != failed)
            service(SetBool, '/'+failed+'/set_available', SetBool.Request(data=False))
            wait(lambda: len(status().get('members', {})) == 2 and
                 failed not in status()['members'])
            assert survivor in status()['members']
            report['replacement'] = dict(status()['pairs'][0]['members'])
            report['disabled'] = failed
            docked(1)
            reset()
            # Two pairs cannot be formed while only three robots are healthy.
            for name in ('robot_4', 'robot_5'):
                service(SetBool, '/'+name+'/set_available', SetBool.Request(data=False))
            wait(lambda: sum(s['healthy'] for s in state['registry']['robots'].values()) == 3)
            count.publish(Int32(data=2))
            wait(lambda: status().get('status') == 'INSUFFICIENT_HEALTHY_ROBOTS')
            assert not status()['members']
            stopped()
            for name in ('robot_4', 'robot_5'):
                service(SetBool, '/'+name+'/set_available', SetBool.Request(data=True))
            docked(2)
            report['insufficient_health_and_rejoin'] = True
            reset()
            print(json.dumps(report, indent=2), flush=True)
            print('PASS: replacement and insufficient-health recovery', flush=True)
            return
        count.publish(Int32(data=1))
        wait(lambda: len(status().get('members', {})) == 2)
        first = dict(status()['members'])
        a, b = list(first)
        expected = [(initial[a][i]+initial[b][i])/2 for i in (0, 1)]
        assert math.dist(status()['pairs'][0]['origin'][:2], expected) < .015
        docked(1)
        finished = {n: xy(n) for n in first}
        report['one_pair'] = {'robots': sorted(first), 'origin': status()['pairs'][0]['origin'],
                              'errors': status()['pairs'][0]['errors']}
        assert all(math.dist(initial[n], xy(n)) < .015 for n in initial if n not in first)
        # Backward-compatible PAIR now really requests one additional pair.
        shape.publish(String(data='PAIR'))
        docked(2)
        assert set(first) <= set(status()['members'])
        assert all(math.dist(finished[n], xy(n)) < .015 for n in first)
        assert len(set(status()['members'])) == 4
        report['additional_pair'] = {'robots': sorted(status()['members']),
                                     'errors': [p['errors'] for p in status()['pairs']]}
        epoch = status()['epoch']
        count.publish(Int32(data=2))
        settle(2.)
        assert status()['epoch'] == epoch and status()['completed_pairs'] == 2
        stopped()
        report['repeat_count_is_idempotent'] = True
        reset()
        assert all(math.dist(initial[n], xy(n)) < .03 for n in initial)
        report['reset_after_docking'] = True
        count.publish(Int32(data=2))
        wait(lambda: len(status().get('members', {})) == 4)
        assert len(status()['pairs']) == 2 and not status()['complete']
        docked(2)
        report['two_pairs_from_one_command'] = [p['members'] for p in status()['pairs']]
        reset()
        count.publish(Int32(data=2))
        wait(lambda: len(status().get('members', {})) == 4)
        settle(2.)
        reset()
        assert all(math.dist(initial[n], xy(n)) < .03 for n in initial)
        report['reset_during_motion'] = True
        # A disabled heartbeat is restored by the environment reset.
        service(SetBool, '/robot_3/set_available', SetBool.Request(data=False))
        reset()
        report['reset_reenables_agents'] = True
        # Deleted Gazebo entities are restored using the retained URDF.
        service(DeleteEntity, '/delete_entity', DeleteEntity.Request(name='robot_5'))
        settle(2.)
        reset()
        assert math.dist(initial['robot_5'], xy('robot_5')) < .03
        report['reset_respawns_deleted_robot'] = True
        print(json.dumps(report, indent=2), flush=True)
        print('PASS: peer-relative pairs and environment reset', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
