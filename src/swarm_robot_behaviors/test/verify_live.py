#!/usr/bin/env python3
"""Opt-in live integration check; run ONLY against a dedicated experiment.

This script drives no manual cmd_vel. With --safety it disables controllers,
spawns temporary obstacles, exercises each controller separately, then leaves
all five controllers disabled. It never runs as part of colcon test.
"""

import argparse
import json
import math
import statistics
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=160.0)
    parser.add_argument('--safety', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_five_robots')
    latest, owners, states = {}, {}, {}
    clock = [0.0]
    subscriptions = []
    buffer = Buffer()
    node.tf_listener = TransformListener(buffer, node)

    def receive(key, msg):
        latest[key] = msg

    def tf_owners(msg):
        for transform in msg.transforms:
            owners.setdefault(transform.child_frame_id, set()).add(transform.header.frame_id)

    subscriptions.append(node.create_subscription(
        Clock, '/clock', lambda msg: clock.__setitem__(
            0, msg.clock.sec + msg.clock.nanosec * 1e-9), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage, '/tf', tf_owners, 100))
    subscriptions.append(node.create_subscription(
        TFMessage, '/tf_static', tf_owners,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    for i in range(1, 6):
        for tag, kind in [('odom', Odometry), ('imu/data', Imu), ('range/front', LaserScan),
                          ('joint_states', JointState), ('cmd_vel', Twist),
                          ('behavior/state', String)]:
            subscriptions.append(node.create_subscription(
                kind, f'/robot_{i}/{tag}',
                lambda msg, key=(i, tag): receive(key, msg), qos_profile_sensor_data))

    def spin_until(predicate, wall_timeout=30):
        end = time.monotonic() + wall_timeout
        while not predicate():
            assert time.monotonic() < end, 'Timed out waiting for simulation data'
            rclpy.spin_once(node, timeout_sec=.05)

    def sim_wait(seconds):
        end = clock[0] + seconds
        spin_until(lambda: clock[0] >= end, max(30, seconds * 5))

    def call(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(request)
        spin_until(future.done)
        response = future.result()
        assert response.success, response
        node.destroy_client(client)
        return response

    def enable(i, value):
        role = 'leader' if i == 1 else 'follower'
        call(SetBool, f'/robot_{i}/{role}_controller/set_enabled', SetBool.Request(data=value))

    def xy(i):
        p = latest[i, 'odom'].pose.pose.position
        return p.x, p.y

    def distance(i, j):
        return math.dist(xy(i), xy(j))

    def yaw(i):
        q = latest[i, 'odom'].pose.pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    report = {}
    target_name = None
    safety_started = False
    try:
        spin_until(lambda: len(latest) == 30 and clock[0] > 0)
        for i in range(1, 6):
            for suffix in ['odom', 'base_footprint', 'base_link', 'left_wheel_link',
                           'right_wheel_link', 'caster_link', 'imu_link', 'front_range_link']:
                frame = f'robot_{i}/{suffix}'
                spin_until(lambda: buffer.can_transform('world', frame, Time()))
            assert latest[i, 'odom'].header.frame_id == f'robot_{i}/odom'
            assert latest[i, 'odom'].child_frame_id == f'robot_{i}/base_footprint'
            assert latest[i, 'imu/data'].header.frame_id == f'robot_{i}/imu_link'
            assert latest[i, 'range/front'].header.frame_id == f'robot_{i}/front_range_link'
            assert len(latest[i, 'range/front'].ranges) == 3
            assert latest[i, 'joint_states'].name == [
                f'robot_{i}/left_wheel_joint', f'robot_{i}/right_wheel_joint']
            assert len(node.get_publishers_info_by_topic(f'/robot_{i}/cmd_vel')) == 1
        report['topics_and_tf'] = '25 required robot topics, 40 uniquely prefixed TF frames'
        start = clock[0]
        before = {i: xy(i) for i in range(1, 6)}
        paths = {i: 0.0 for i in before}
        spacings = {i: [] for i in range(2, 6)}
        min_separation = math.inf
        next_print = start
        while clock[0] < start + args.seconds:
            sim_wait(.5)
            for i in range(1, 6):
                cmd = latest[i, 'cmd_vel']
                assert 0 <= cmd.linear.x <= .200001
                assert abs(cmd.angular.z) <= (.700001 if i == 1 else .800001)
                assert max(abs(value) for value in xy(i)) < 4.85, (i, xy(i))
                paths[i] += math.dist(xy(i), before[i])
                before[i] = xy(i)
                states.setdefault(i, set()).add(latest[i, 'behavior/state'].data)
                if i > 1 and clock[0] - start > min(60, args.seconds / 2):
                    spacings[i].append(distance(i, i-1))
                for j in range(1, i):
                    min_separation = min(min_separation, distance(i, j))
            assert all(len(publishers) == 1 for publishers in owners.values()), owners
            if clock[0] >= next_print:
                print(json.dumps({'elapsed_sim_s': round(clock[0]-start, 1),
                                  'spacing_m': [round(distance(i, i-1), 3) for i in range(2, 6)],
                                  'leader_state': latest[1, 'behavior/state'].data}), flush=True)
                next_print += 30
        assert all(length > args.seconds * .035 for length in paths.values()), paths
        for i in range(2, 6):
            assert 'following' in states[i], states
            assert .45 < statistics.mean(spacings[i]) < 1.05, spacings[i]
        assert min_separation > .15, min_separation
        report['path_lengths_m'] = paths
        report['mean_spacing_m_after_settling'] = {
            i: statistics.mean(values) for i, values in spacings.items()}
        report['minimum_pair_separation_m'] = min_separation
        report['leader_states_seen'] = sorted(states[1])
        if args.safety:
            safety_started = True
            for i in range(1, 6):
                enable(i, False)
            sim_wait(1)
            report['safety'] = {}
            for i in range(1, 6):
                x, y = xy(i)
                heading = yaw(i)
                target_name = f'verification_obstacle_{i}'
                request = SpawnEntity.Request(name=target_name)
                request.xml = '''<sdf version="1.6"><model name="target"><static>true</static>
<link name="wall"><collision name="collision"><geometry><box><size>0.1 0.4 0.2</size>
</box></geometry></collision><visual name="visual"><geometry><box><size>0.1 0.4 0.2</size>
</box></geometry></visual></link></model></sdf>'''
                request.initial_pose.position.x = x + .282 * math.cos(heading)
                request.initial_pose.position.y = y + .282 * math.sin(heading)
                request.initial_pose.position.z = .1
                request.initial_pose.orientation.z = math.sin(heading/2)
                request.initial_pose.orientation.w = math.cos(heading/2)
                call(SpawnEntity, '/spawn_entity', request)
                spin_until(lambda: min(latest[i, 'range/front'].ranges) < .25)
                reading = min(latest[i, 'range/front'].ranges)
                enable(i, True)
                spin_until(lambda: latest[i, 'behavior/state'].data == 'avoiding')
                cmd = latest[i, 'cmd_vel']
                assert cmd.linear.x == 0 and .1 < cmd.angular.z <= .7, cmd
                sim_wait(.8)
                turned = math.atan2(math.sin(yaw(i)-heading), math.cos(yaw(i)-heading))
                assert turned > .2, turned
                report['safety'][i] = {'range_m': reading, 'turn_rad': turned}
                call(DeleteEntity, '/delete_entity', DeleteEntity.Request(name=target_name))
                target_name = None
                spin_until(lambda: latest[i, 'behavior/state'].data != 'avoiding')
                enable(i, False)
                sim_wait(.4)
            print('PASS: all five controllers stopped forward motion and turned.', flush=True)
        print(json.dumps(report, indent=2), flush=True)
        print('PASS: five-robot experiment', flush=True)
    finally:
        if target_name:
            call(DeleteEntity, '/delete_entity', DeleteEntity.Request(name=target_name))
        if safety_started:
            for i in range(1, 6):
                enable(i, False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
