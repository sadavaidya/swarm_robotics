"""Stop controllers, restore configured spawn poses, and clear swarm task state."""

import json
import math
import threading
import time

from gazebo_msgs.srv import SetEntityState, SpawnEntity
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .protocol import decode, LATCHED, now_seconds, setting, wrap


class EnvironmentReset(Node):
    def __init__(self):
        super().__init__('environment_reset')
        self.robots = json.loads(setting(self, 'robots_json', '[]'))
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.states, self.poses, self.descriptions = {}, {}, {}
        self.subscriptions_kept = []
        self.agent_clients = {}
        self.hold = self.create_client(SetBool, '/swarm/hold', callback_group=self.group)
        self.detach_before_reset = bool(setting(self, 'detach_before_reset', False))
        self.dock_state = {}
        self.dock_subscription = self.create_subscription(
            String, '/swarm/dock_status', self.on_docks, LATCHED, callback_group=self.group)
        self.pose_service = '/swarm/simulation/set_entity_state'
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity', callback_group=self.group)
        for robot in self.robots:
            name = robot['id']
            self.agent_clients[name] = self.create_client(
                Trigger, '/' + name + '/swarm/reset', callback_group=self.group)
            self.subscriptions_kept.extend([
                self.create_subscription(String, '/' + name + '/swarm/local_state',
                                         lambda msg, n=name: self.on_state(n, msg), 10,
                                         callback_group=self.group),
                self.create_subscription(Odometry, '/' + name + '/odom',
                                         lambda msg, n=name: self.poses.update({n: msg}), 10,
                                         callback_group=self.group),
                self.create_subscription(String, '/' + name + '/robot_description',
                                         lambda msg, n=name: self.descriptions.update(
                                             {n: msg.data}),
                                         LATCHED, callback_group=self.group),
            ])
        self.service = self.create_service(Trigger, '/swarm/reset', self.reset,
                                           callback_group=self.group)

    def on_docks(self, msg):
        value = decode(msg)
        if value:
            self.dock_state = value

    def docks_released(self, since):
        epochs = {self.states.get(r['id'], {}).get('epoch') for r in self.robots}
        state = self.dock_state
        return (len(epochs) == 1 and None not in epochs and
                state.get('epoch') in epochs and state.get('physical') is True and
                state.get('bonds') == [] and state.get('stamp', -1) > since)

    def release_docks(self):
        """Fresh RPC endpoint plus fresh current-epoch physical confirmation."""
        client = self.create_client(Trigger, '/swarm/detach_all', callback_group=self.group)
        future = None
        since = now_seconds(self)
        try:
            if not client.wait_for_service(timeout_sec=8.):
                raise RuntimeError('Physical latch plugin unavailable')
            future = client.call_async(Trigger.Request())
            self.wait_for(lambda: future.done() or self.docks_released(since), 8.,
                          'Physical latch release not confirmed')
            if self.docks_released(since):
                return
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError('Physical latch release failed')
            self.wait_for(lambda: self.docks_released(since), 8.,
                          'Fresh physical inventory did not confirm latch release')
        finally:
            if future is not None and not future.done():
                future.cancel()
            self.destroy_client(client)

    def on_state(self, name, msg):
        value = decode(msg)
        if value:
            self.states[name] = value

    @staticmethod
    def wait_for(predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.05)
        raise RuntimeError(reason)

    def call(self, client, request, timeout=8.):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('Service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        try:
            self.wait_for(future.done, timeout, 'Service timed out: ' + client.srv_name)
        except RuntimeError:
            future.cancel()
            raise
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Service failed: ' + client.srv_name)
        return response

    @staticmethod
    def initial_pose(robot):
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = robot['x'], robot['y'], .01
        pose.orientation.z = math.sin(robot['yaw']/2)
        pose.orientation.w = math.cos(robot['yaw']/2)
        return pose

    def restored(self, robot, since):
        msg = self.poses.get(robot['id'])
        if msg is None:
            return False
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return (stamp > since and math.hypot(p.x-robot['x'], p.y-robot['y']) < .03
                and abs(wrap(yaw-robot['yaw'])) < .06)

    def restore_robot(self, robot):
        """Set a held robot's pose, accepting fresh physical confirmation too.

        Under load, a SetEntityState reply can be delayed even after Gazebo has
        applied the pose. Do not fail the whole reset when fresh odometry proves
        the requested state. An unconfirmed request is retried a bounded number
        of times; all attempts occur while the fleet's motion hold is active.
        """
        req = SetEntityState.Request()
        req.state.name = robot['id']
        req.state.pose = self.initial_pose(robot)
        req.state.reference_frame = 'world'
        for attempt in range(3):
            # Fresh RPC endpoints avoid a stalled long-lived SetEntityState
            # client preventing later robots from being restored under load.
            client = self.create_client(SetEntityState, self.pose_service,
                                        callback_group=self.group)
            future = None
            try:
                if not client.wait_for_service(timeout_sec=8.):
                    raise RuntimeError('Gazebo state plugin unavailable')
                since = now_seconds(self)
                future = client.call_async(req)
                try:
                    self.wait_for(lambda: future.done() or self.restored(robot, since), 8.,
                                  'Gazebo pose reset timed out for '+robot['id'])
                except RuntimeError:
                    if attempt == 2:
                        raise
                    continue
                if not future.done():
                    # Fresh odometry independently confirms the physical reset
                    # if its RPC reply arrives late.
                    return
                result = future.result()
            finally:
                if future is not None and not future.done():
                    future.cancel()
                self.destroy_client(client)
            if result is not None and result.success:
                return
            if result is not None:
                # Restore a deleted robot from its retained RSP description,
                # including any optional rear docking geometry.
                xml = self.descriptions.get(robot['id'])
                if not xml:
                    raise RuntimeError('No robot description to restore '+robot['id'])
                spawn = SpawnEntity.Request(name=robot['id'], xml=xml,
                                            robot_namespace='/'+robot['id'],
                                            initial_pose=self.initial_pose(robot),
                                            reference_frame='world')
                self.call(self.spawn, spawn, 15.)
                return
        raise RuntimeError('Gazebo pose reset unconfirmed for '+robot['id'])

    def reset(self, request, response):
        if not self.lock.acquire(blocking=False):
            response.success, response.message = False, 'Reset already in progress'
            return response
        try:
            since = now_seconds(self)
            self.call(self.hold, SetBool.Request(data=True))

            def stopped():
                return all(self.states.get(r['id'], {}).get('state') == 'RESETTING'
                           and self.states[r['id']].get('stamp', -1) >= since
                           for r in self.robots)

            self.wait_for(stopped, 8., 'Controllers did not acknowledge stop')
            if self.detach_before_reset:
                self.release_docks()
            for robot in self.robots:
                self.restore_robot(robot)
                self.call(self.agent_clients[robot['id']], Trigger.Request())
            restored_since = now_seconds(self)
            self.wait_for(lambda: all(self.restored(r, restored_since) for r in self.robots),
                          10., 'Fresh odometry did not confirm restored starting poses')
            self.call(self.hold, SetBool.Request(data=False))
            self.wait_for(
                lambda: all(self.states.get(r['id'], {}).get('state') == 'AVAILABLE'
                            for r in self.robots),
                8., 'Controllers did not become available')
            response.success = True
            response.message = 'Task cleared; all robots stopped at starting poses and available'
            self.get_logger().info(response.message)
        except Exception as error:
            # Fail closed: retain the motion hold if any reset step fails.
            try:
                self.call(self.hold, SetBool.Request(data=True), 2.)
            except Exception:
                pass
            response.success, response.message = False, str(error) + '; motion held; retry reset'
            self.get_logger().error(response.message)
        finally:
            self.lock.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentReset()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
