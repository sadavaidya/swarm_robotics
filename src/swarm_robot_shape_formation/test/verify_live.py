#!/usr/bin/env python3
"""Opt-in full Gazebo test. Moves and resets only a dedicated ten-robot fleet."""
import argparse
from itertools import combinations
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from swarm_robot_docking.protocol import decode, LATCHED, wrap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shapes', nargs='*', choices=['LINE', 'SQUARE', 'CIRCLE'],
                        default=['LINE', 'SQUARE', 'CIRCLE'],
                        help='Shapes to test; an empty list checks interfaces, STOP and reset')
    parser.add_argument('--reconfigure', action='store_true',
                        help='Change connected shapes without resetting between requests')
    parser.add_argument('--check-autonomy', action='store_true')
    parser.add_argument('--check-watchdog', action='store_true')
    parser.add_argument('--report-path', type=Path)
    parser.add_argument('--dds-native', action='store_true',
                        help='Retain the native transport when launch uses dds_udp:=false')
    args = parser.parse_args()
    if (args.check_autonomy or args.check_watchdog) and 'ROS_DOMAIN_ID' not in os.environ:
        parser.error('Fault tests require an explicit dedicated ROS_DOMAIN_ID')
    if (args.check_autonomy or args.check_watchdog) and not args.shapes:
        parser.error('Fault tests require at least one shape')
    if not args.dds_native:
        os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(Path(
            get_package_share_directory('swarm_robot_shape_formation'),
            'config', 'fastdds_udp.xml')))
    rclpy.init()
    node = rclpy.create_node('verify_ten_robot_shapes')
    state, commands, odom = {}, {}, {}
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    names = [f'robot_{i}' for i in range(1, 11)]
    events, motion_check = [], dict(enabled=False, max_step=0., errors=[])

    def receive(kind, msg):
        value = decode(msg)
        state[kind] = value
        if kind in ('shape_goal', 'dock_status') and value:
            stage = (value.get('transition', {}).get('stage') if kind == 'shape_goal'
                     else value.get('transition_stage'))
            event = dict(kind=kind, epoch=value.get('epoch'), stage=stage,
                         shape=value.get('shape'), bonds=len(value.get('bonds', [])))
            if not events or events[-1] != event:
                events.append(event)

    def receive_odom(name, msg):
        previous = odom.get(name)
        if motion_check['enabled'] and previous:
            a, b = previous.pose.pose.position, msg.pose.pose.position
            dt = ((msg.header.stamp.sec-previous.header.stamp.sec) +
                  (msg.header.stamp.nanosec-previous.header.stamp.nanosec)/1e9)
            jump = math.hypot(a.x-b.x, a.y-b.y)
            motion_check['max_step'] = max(motion_check['max_step'], jump)
            if dt > 0. and jump > .2*dt+.015:
                motion_check['errors'].append(dict(robot=name, jump=jump, dt=dt))
        odom[name] = msg

    subscriptions = [node.create_subscription(
        String, '/swarm/'+key, lambda m, k=key: receive(k, m), LATCHED)
        for key in ('formation_status', 'shape_goal', 'shape_request_status', 'dock_status')]
    for name in names:
        subscriptions.extend([
            node.create_subscription(Twist, '/'+name+'/cmd_vel',
                                     lambda m, n=name: commands.update({n: m}), 10),
            node.create_subscription(Odometry, '/'+name+'/odom',
                                     lambda m, n=name: receive_odom(n, m),
                                     qos_profile_sensor_data),
        ])
    report, suspended = {}, []
    replacement_goal = None
    minimum_spacing = math.inf

    def save_report():
        if args.report_path:
            args.report_path.parent.mkdir(parents=True, exist_ok=True)
            args.report_path.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')

    def status():
        return state.get('formation_status', {})

    def wait(predicate, seconds=40.):
        deadline = time.monotonic()+seconds
        while not predicate():
            assert time.monotonic() < deadline, json.dumps(status())
            rclpy.spin_once(node, timeout_sec=.02)

    def settle(seconds=1.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)

    def service(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=10.), name
        future = client.call_async(request)
        wait(future.done, 90.)
        result = future.result()
        assert result and result.success, str(result)
        node.destroy_client(client)
        settle(.5)

    def stopped():
        settle(.6)
        assert len(commands) == 10
        assert all(c.linear.x == c.angular.z == 0 for c in commands.values())

    def cli(name):
        process = subprocess.Popen(
            ['ros2', 'topic', 'pub', '--once', '--keep-alive', '2', '--qos-durability',
             'volatile', '/swarm/shape_command', 'std_msgs/msg/String', "{data: '"+name+"'}"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        wait(lambda: process.poll() is not None, 25.)
        output = process.communicate()[0]
        assert process.returncode == 0, output
        print(output, flush=True)
        wait(lambda: state.get('shape_goal', {}).get('shape') == name)

    def owned_pid(executable, namespace=None):
        domain = ('ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID']).encode()
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                cmd = (entry/'cmdline').read_bytes().split(b'\0')
                env = (entry/'environ').read_bytes().split(b'\0')
                if (domain in env and any(c.endswith(('/'+executable).encode()) for c in cmd)
                        and (namespace is None or ('__ns:=/'+namespace).encode() in cmd)):
                    return int(entry.name)
            except (FileNotFoundError, PermissionError):
                pass
        raise AssertionError('No owned process: '+executable)

    def spacing():
        return min(math.hypot(a.pose.pose.position.x-b.pose.pose.position.x,
                              a.pose.pose.position.y-b.pose.pose.position.y)
                   for a, b in combinations(odom.values(), 2))

    try:
        wait(lambda: len(odom) == len(commands) == 10 and
             len(status().get('robots', {})) == 10)
        service(Trigger, '/swarm/reset', Trigger.Request())
        wait(lambda: all(p.get('healthy') for p in status()['robots'].values()))
        stopped()
        initial = {n: [m.pose.pose.position.x, m.pose.pose.position.y] for n, m in odom.items()}
        assert spacing() >= .78
        minimum_spacing = spacing()
        report['starting_poses'] = initial
        for name in names:
            for frame in ('front_dock_frame', 'rear_dock_frame', 'imu_link', 'front_range_link'):
                wait(lambda n=name, f=frame: buffer.can_transform('world', n+'/'+f, Time()))
            assert len(node.get_publishers_info_by_topic('/'+name+'/cmd_vel')) == 1
        report['interfaces_and_rear_frames'] = 'PASS'
        for index, shape in enumerate(args.shapes):
            if index and not args.reconfigure:
                service(Trigger, '/swarm/reset', Trigger.Request())
            maximum_concurrent = 0
            event_start = len(events)
            previous_bonds = len(state.get('dock_status', {}).get('bonds', []))
            motion_check.update(enabled=bool(index and args.reconfigure), max_step=0., errors=[])
            cli(shape)
            wait(lambda: len(status().get('members', {})) == 10, 90.)
            first_members = dict(status()['members'])
            assert len({s['id'] for s in first_members.values()}) == 10
            if args.check_watchdog and index == 0:
                wait(lambda: any(c.linear.x > .04 for c in commands.values()), 120.)
                mover = next(n for n, c in commands.items() if c.linear.x > .04)
                pid = owned_pid('shape_agent', mover)
                os.kill(pid, signal.SIGSTOP)
                suspended.append(pid)
                settle(1.)
                assert commands[mover].linear.x == commands[mover].angular.z == 0
                os.kill(pid, signal.SIGCONT)
                suspended.remove(pid)
                report['independent_watchdog'] = mover
            if args.check_autonomy and index == len(args.shapes)-1:
                pid = owned_pid('shape_goal')
                # Remove the high-level computer while onboard agents assemble.
                # Restart a fresh DDS context afterward: prolonged SIGSTOP can
                # invalidate Fast DDS peer discovery in an existing context.
                os.kill(pid, signal.SIGINT)
                wait(lambda: not Path('/proc', str(pid)).exists(), 20.)
                report['goal_publisher_stopped_during_assembly'] = True
            start = time.monotonic()
            next_print = start
            while not status().get('complete'):
                settle(.15)
                assert time.monotonic()-start < 900., json.dumps(status())
                minimum_spacing = min(minimum_spacing, spacing())
                maximum_concurrent = max(maximum_concurrent,
                                         len(status().get('moving_robots', [])))
                assert minimum_spacing > .12, minimum_spacing
                assert all(abs(c.linear.x) <= .15001 and abs(c.angular.z) <= .60001
                           for c in commands.values())
                if time.monotonic() >= next_print:
                    print(json.dumps(dict(shape=shape, elapsed=round(time.monotonic()-start),
                                          status=status().get('status'),
                                          dock_pending=state.get('dock_status', {}).get('pending'),
                                          phases={n: p['state'] for n, p in
                                                  status().get('robots', {}).items()})),
                          flush=True)
                    next_print += 20.
            stopped()
            motion_check['enabled'] = False
            if index and args.reconfigure:
                trace = events[event_start:]
                assert any(e['kind'] == 'dock_status' and e['stage'] == 'HOLD' and
                           e['bonds'] == previous_bonds for e in trace), trace
                assert any(e['kind'] == 'dock_status' and e['stage'] == 'RELEASE' and
                           e['bonds'] == 0 for e in trace), trace
                assert not motion_check['errors'], motion_check['errors']
                report.setdefault('automatic_shape_changes', []).append(dict(
                    to=shape, old_bonds=previous_bonds, stopped_before_release=True,
                    release_confirmed=True, no_teleport=True,
                    max_odometry_step=motion_check['max_step']))
            assert maximum_concurrent >= 2, 'Concurrent motion was never observed'
            assert status()['mechanical_attachment'] is True
            assert len(status()['bonds']) == (9 if shape == 'LINE' else 10)
            assert status()['members'] == first_members
            actual_errors = {}
            for name, member in status()['members'].items():
                msg, target = odom[name], member['target']
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                error = dict(position=math.hypot(p.x-target[0], p.y-target[1]),
                             yaw=abs(wrap(yaw-target[2])))
                assert error['position'] < .05 and error['yaw'] < .25, error
                actual_errors[name] = error
            report[shape] = dict(bonds=status()['bonds'], maximum_concurrent=maximum_concurrent,
                                 seconds=round(time.monotonic()-start, 1), errors=actual_errors,
                                 members={n: s['id'] for n, s in first_members.items()})
            report['minimum_sampled_center_spacing'] = minimum_spacing
            save_report()
            print('PASS: '+shape+' complete; all ten stopped', flush=True)
            for pid in suspended:
                os.kill(pid, signal.SIGCONT)
            suspended.clear()
        if args.check_autonomy:
            restart_env = dict(os.environ)
            replacement_goal = subprocess.Popen(
                ['ros2', 'run', 'swarm_robot_shape_formation', 'shape_goal',
                 '--ros-args', '-p', 'use_sim_time:=false'],
                env=restart_env, start_new_session=True)
            wait(lambda: state.get('shape_goal', {}).get('shape') == 'IDLE', 40.)
            service(SetBool, '/swarm/hold', SetBool.Request(data=False))
            report['goal_publisher_restart'] = True
        cli('STOP')
        stopped()
        service(Trigger, '/swarm/reset', Trigger.Request())
        assert all(math.dist(initial[n], [m.pose.pose.position.x, m.pose.pose.position.y]) < .03
                   for n, m in odom.items())
        stopped()
        wait(lambda: state.get('dock_status', {}).get('bonds') == [])
        report['reset_releases_physical_latches'] = True
        report['reset_restores_starting_layout'] = True
        # Invalid shapes must be rejected without starting another task.
        subprocess.run(['ros2', 'topic', 'pub', '--once', '--keep-alive', '2',
                        '--qos-durability', 'volatile', '/swarm/shape_command',
                        'std_msgs/msg/String', "{data: 'HEXAGON'}"],
                       capture_output=True, timeout=25., check=True)
        wait(lambda: state.get('shape_request_status', {}).get('accepted') is False)
        stopped()
        report['minimum_sampled_center_spacing'] = minimum_spacing
        save_report()
        print(json.dumps(report, indent=2), flush=True)
        label = 'shape formation integration' if args.shapes else 'interfaces, STOP and reset'
        print('PASS: ten-robot '+label, flush=True)
    finally:
        for pid in suspended:
            os.kill(pid, signal.SIGCONT)
        if replacement_goal is not None and replacement_goal.poll() is None:
            os.killpg(replacement_goal.pid, signal.SIGINT)
            try:
                replacement_goal.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(replacement_goal.pid, signal.SIGTERM)
                replacement_goal.wait(timeout=15.)
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
