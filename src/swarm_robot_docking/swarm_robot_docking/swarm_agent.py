"""One identical robot-side health publisher and slot bidder per namespace."""

import math

from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformException

from .protocol import decode, encode, fresh, LATCHED, now_seconds, number, run, setting
from .pose_provider import PoseProvider
from .shape_library import bid_cost


class SwarmAgent(Node):
    def __init__(self):
        super().__init__('swarm_agent')
        self.robot = setting(self, 'robot_id', self.get_namespace().strip('/'))
        if not self.robot:
            raise ValueError('robot_id or a robot namespace is required')
        self.timeout = number(self, 'data_timeout', .7)
        self.weight = number(self, 'heading_weight', .08)
        self.enabled = True
        self.shape = None
        self.controller = None
        self.poses = PoseProvider(self, [self.robot], 'world', self.timeout)
        self.registry_pub = self.create_publisher(String, '/swarm/robot_state', 20)
        self.health_pub = self.create_publisher(String, 'swarm/health', 10)
        self.shape_sub = self.create_subscription(String, '/swarm/shape', self.on_shape, LATCHED)
        self.state_sub = self.create_subscription(String, 'swarm/local_state', self.on_state, 10)
        self.service = self.create_service(SetBool, 'set_available', self.set_available)
        self.reset_service = self.create_service(Trigger, 'swarm/reset', self.reset)
        self.timer = self.create_timer(1 / number(self, 'heartbeat_rate', 4.0), self.tick)

    def on_shape(self, msg):
        value = decode(msg)
        if value and isinstance(value.get('slots'), list):
            self.shape = value

    def on_state(self, msg):
        value = decode(msg)
        if value and value.get('robot_id') == self.robot:
            self.controller = value

    def set_available(self, request, response):
        self.enabled = request.data
        self.health_pub.publish(encode({'stamp': now_seconds(self), 'enabled': self.enabled}))
        response.success = True
        response.message = ('heartbeat resumed' if self.enabled else
                            'heartbeat suppressed; local motion disabled; registry will time out')
        self.get_logger().info(response.message)
        return response

    def reset(self, request, response):
        self.poses.clear()
        self.controller = None
        self.enabled = True
        response.success, response.message = True, 'Agent cache cleared; heartbeat enabled'
        return response

    def tick(self):
        now = now_seconds(self)
        self.health_pub.publish(encode({'stamp': now, 'enabled': self.enabled}))
        if not self.enabled:
            return
        pose = self.poses.get(self.robot, now)
        controller_ok = self.controller and fresh(self.controller.get('stamp'), now, self.timeout)
        healthy = pose is not None and bool(controller_ok)
        dock = None
        if pose is not None:
            try:
                tf = self.poses.buffer.lookup_transform(
                    'world', self.robot + '/front_dock_frame', Time())
                p, q = tf.transform.translation, tf.transform.rotation
                dock = [p.x, p.y, math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))]
            except TransformException:
                pass
        xy_yaw = [pose.x, pose.y, pose.yaw] if pose else None
        bids = {}
        if healthy and self.shape:
            bids = {slot['id']: bid_cost(xy_yaw, slot, self.weight)
                    for slot in self.shape['slots']}
        local = self.controller or {}
        self.registry_pub.publish(encode({
            'robot_id': self.robot, 'stamp': now, 'alive': True, 'healthy': healthy,
            'available': healthy and local.get('state') == 'AVAILABLE',
            'pose': xy_yaw, 'dock': dock, 'bids': bids,
            'shape_epoch': self.shape.get('epoch') if self.shape else None,
            'state': local.get('state', 'IDLE') if healthy else 'FAILED',
            'assignment_epoch': local.get('epoch'), 'generation': local.get('generation'),
        }))


def main(args=None):
    run(SwarmAgent, args)
