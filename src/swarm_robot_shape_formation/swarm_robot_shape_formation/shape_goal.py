"""Publish shape tasks and coordinate stop/release barriers between tasks."""
import time
import uuid

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from swarm_robot_docking.protocol import decode, encode, LATCHED, run, setting

from .shapes import make_shape


class ShapeGoal(Node):
    def __init__(self):
        super().__init__('shape_goal')
        self.count = int(setting(self, 'robot_count', 10))
        self.spacing = float(setting(self, 'spacing', .22))
        self.names = list(setting(self, 'robot_ids', [f'robot_{i+1}' for i in range(self.count)]))
        self.transition_timeout = float(setting(self, 'transition_timeout', 30.))
        self.goal_pub = self.create_publisher(String, '/swarm/shape_goal', LATCHED)
        self.result_pub = self.create_publisher(String, '/swarm/shape_request_status', LATCHED)
        self.command_sub = self.create_subscription(
            String, '/swarm/shape_command', self.command, 10)
        self.hold_service = self.create_service(SetBool, '/swarm/hold', self.hold)
        self.docks, self.docks_received, self.states, self.commands = {}, -1., {}, {}
        self.dock_sub = self.create_subscription(
            String, '/swarm/dock_status', self.on_docks, LATCHED)
        self.robot_subs = []
        for name in self.names:
            self.robot_subs.extend([
                self.create_subscription(String, '/'+name+'/swarm/local_state',
                                         lambda msg, n=name: self.on_state(n, msg), 10),
                self.create_subscription(Twist, '/'+name+'/cmd_vel',
                                         lambda msg, n=name: self.on_command(n, msg), 10),
            ])
        self.resetting, self.transition, self.current_goal = False, None, None
        self.publish('IDLE', [])
        # Wall time keeps STOP, cancellation and timeout responsive during pause.
        self.timer = self.create_timer(1., self.rebroadcast)
        self.transition_timer = self.create_timer(.1, self.advance_transition)

    def on_docks(self, msg):
        value = decode(msg)
        if value:
            self.docks, self.docks_received = value, time.monotonic()

    def on_state(self, name, msg):
        value = decode(msg)
        if value:
            self.states[name] = (value, time.monotonic())

    def on_command(self, name, msg):
        self.commands[name] = (msg.linear.x, msg.angular.z, time.monotonic())

    def publish(self, name, slots, **extra):
        self.current_goal = dict(epoch=uuid.uuid4().hex, shape=name,
                                 slots=slots, resetting=self.resetting, **extra)
        self.rebroadcast()

    def rebroadcast(self):
        self.goal_pub.publish(encode(self.current_goal))

    def notice(self, name, accepted, notice, **extra):
        self.result_pub.publish(encode(dict(
            accepted=accepted, shape=name, notice=notice, **extra)))
        logger = self.get_logger()
        (logger.info if accepted else logger.warning)(name+': '+notice)

    def command(self, msg):
        name = msg.data.strip().upper()
        try:
            if self.resetting:
                raise ValueError('Reset in progress; retry when reset completes')
            slots = [] if name == 'STOP' else make_shape(name, self.count, self.spacing)
            if name == 'STOP':
                self.transition = None
                self.publish('STOP', [])
                self.notice(name, True, 'Stopped; pending shape change cancelled')
            elif self.transition and self.transition['stage'] != 'FAILED':
                self.transition.update(name=name, slots=slots)
                self.current_goal['transition']['requested_shape'] = name
                self.rebroadcast()
                self.notice(name, True, 'Queued; latest shape follows the stop/release barrier',
                            phase=self.transition['stage'])
            elif self.current_goal['shape'] == name and self.current_goal['slots']:
                self.notice(name, True, 'This shape is already active')
            elif self.current_goal['shape'] == 'IDLE' and not self.docks.get('bonds'):
                self.publish(name, slots)
                self.notice(name, True, 'Goal broadcast; robots negotiate roles with peers')
            else:
                now = time.monotonic()
                self.transition = dict(name=name, slots=slots, stage='HOLD', started=now,
                                       stopped_since=None, release_started=None)
                self.publish('TRANSITION', [], transition=dict(stage='HOLD', requested_shape=name))
                self.notice(name, True, 'Stopping the fleet before releasing the current shape',
                            phase='HOLD')
        except ValueError as error:
            self.notice(name, False, str(error))

    def stopped(self, now):
        epoch, start = self.current_goal['epoch'], self.transition['started']
        for name in self.names:
            state, received = self.states.get(name, ({}, -1.))
            v, w, command_received = self.commands.get(name, (1., 1., -1.))
            if (state.get('epoch') != epoch or state.get('state') != 'TRANSITION_HOLD' or
                    received < start or now-received > 1.5 or
                    command_received < start or now-command_received > 1.5 or
                    abs(v)+abs(w) > 1e-6):
                return False
        return True

    def advance_transition(self):
        transition = self.transition
        if not transition or transition['stage'] == 'FAILED':
            return
        now = time.monotonic()
        if now-transition['started'] > self.transition_timeout:
            transition['stage'] = self.current_goal['transition']['stage'] = 'FAILED'
            self.rebroadcast()
            self.notice(transition['name'], False,
                        'Shape change timed out; fleet held. Resend a shape to retry.',
                        phase='FAILED')
            return
        if not self.stopped(now):
            transition['stopped_since'] = None
            return
        if transition['stopped_since'] is None:
            transition['stopped_since'] = now
        # Let the wheel controller brake; commands/ACKs must remain current.
        if now-transition['stopped_since'] < .6:
            return
        if transition['stage'] == 'HOLD':
            transition['stage'] = self.current_goal['transition']['stage'] = 'RELEASE'
            transition['release_started'] = now
            self.rebroadcast()
            self.notice(transition['name'], True, 'Fleet stopped; releasing physical latches',
                        phase='RELEASE')
            return
        if (self.docks.get('epoch') == self.current_goal['epoch'] and
                self.docks.get('transition_stage') == 'RELEASE' and
                self.docks.get('physical') is True and self.docks.get('bonds') == [] and
                self.docks_received >= transition['release_started'] and
                now-self.docks_received < 1.5):
            self.transition = None
            self.publish(transition['name'], transition['slots'], reconfigure=True)
            self.notice(transition['name'], True,
                        'Latches released; forming the new shape from current positions',
                        phase='FORMING')

    def hold(self, request, response):
        self.transition = None
        self.resetting = request.data
        self.publish('IDLE', [])
        response.success, response.message = True, 'Motion held' if request.data else 'Ready'
        return response


def main(args=None):
    run(ShapeGoal, args)
