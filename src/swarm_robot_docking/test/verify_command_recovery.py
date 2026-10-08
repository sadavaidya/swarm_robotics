#!/usr/bin/env python3
"""Opt-in: one-shot CLI commands, capacity clamping and Gazebo clock rewind."""
import json
import subprocess
import time

from geometry_msgs.msg import Twist
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from swarm_robot_docking.protocol import decode, LATCHED


def main():
    rclpy.init()
    node = rclpy.create_node('verify_pair_command_recovery')
    states, commands = {}, {}
    subscriptions = [node.create_subscription(
        String, '/swarm/'+key, lambda msg, k=key: states.update({k: decode(msg)}), LATCHED)
        for key in ('docking_status', 'registry')]
    for i in range(1, 6):
        name = f'robot_{i}'
        subscriptions.append(node.create_subscription(
            Twist, '/'+name+'/cmd_vel', lambda m, n=name: commands.update({n: m}), 10))

    def status():
        return states.get('docking_status', {})

    def wait(predicate, timeout=30.):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < deadline, json.dumps(status())
            rclpy.spin_once(node, timeout_sec=.02)

    def settle(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def call(kind, topic):
        client = node.create_client(kind, topic)
        assert client.wait_for_service(timeout_sec=8.), topic
        future = client.call_async(kind.Request())
        wait(future.done)
        response = future.result()
        assert response is not None
        if hasattr(response, 'success'):
            assert response.success, response.message
        node.destroy_client(client)

    def cli(count):
        # Match the reliable volatile command reader and allow discovery/delivery
        # to finish before Humble tears down its short-lived CLI publisher.
        result = subprocess.run(
            ['ros2', 'topic', 'pub', '--once', '--keep-alive', '2',
             '--qos-durability', 'volatile', '/swarm/pair_count',
             'std_msgs/msg/Int32', '{data: '+str(count)+'}'],
            capture_output=True, text=True, timeout=20., check=True)
        print(result.stdout, flush=True)
        wait(lambda: status().get('input_pair_count') == count)

    def docked():
        wait(lambda: status().get('complete') and status().get('completed_pairs') == 2, 180.)
        settle(.6)
        assert len(status()['members']) == 4
        assert len(commands) == 5
        assert all(c.linear.x == c.angular.z == 0 for c in commands.values())

    report = {}
    try:
        wait(lambda: len(states.get('registry', {}).get('robots', {})) == 5)
        call(Trigger, '/swarm/reset')
        wait(lambda: status().get('requested_pairs') == 0)
        cli(3)
        assert status()['requested_pairs'] == 2 and '3 pairs not possible' in status()['notice']
        docked()
        report['cli_count_3'] = dict(
            accepted=status()['requested_pairs'], notice=status()['notice'],
            robots=sorted(status()['members']))
        print('PASS: --once data=3 formed two pairs and stopped all five', flush=True)
        old_epoch = status()['epoch']
        old_time = status()['stamp']
        call(Empty, '/reset_simulation')
        wait(lambda: status().get('epoch') != old_epoch and
             status().get('stamp', old_time) < old_time)
        assert status()['requested_pairs'] == 2 and not status()['resetting']
        docked()
        report['clock_rewind'] = 'Recovered and completed two pairs without republishing'
        print('PASS: clock rewind recovered without a reset hold', flush=True)
        call(Trigger, '/swarm/reset')
        wait(lambda: status().get('requested_pairs') == 0)
        cli(2)
        docked()
        report['cli_count_2_after_reset'] = 'Two pairs completed'
        call(Trigger, '/swarm/reset')
        print(json.dumps(report, indent=2), flush=True)
        print('PASS: pair command and clock recovery integration', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
