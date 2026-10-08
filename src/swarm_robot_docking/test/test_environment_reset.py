"""Lost RPC replies must not hide a confirmed reset or release a failed hold."""
from types import SimpleNamespace
import threading
from unittest.mock import Mock

from gazebo_msgs.srv import SetEntityState, SpawnEntity
from rclpy.task import Future
from std_srvs.srv import Trigger
import pytest

from swarm_robot_docking import environment_reset
from swarm_robot_docking.environment_reset import EnvironmentReset


ROBOT = dict(id='robot_1', x=1., y=-2., yaw=.5)


def completed(success):
    future = Future()
    future.set_result(SetEntityState.Response(success=success))
    return future


def resetter(monkeypatch, futures, confirmed=False):
    node = EnvironmentReset.__new__(EnvironmentReset)
    monkeypatch.setattr(environment_reset, 'now_seconds', lambda _: 10.)
    node.detach_before_reset = False
    node.pose_service, node.group = '/swarm/simulation/set_entity_state', object()
    node.rpc_clients = [SimpleNamespace(wait_for_service=Mock(return_value=True),
                                        call_async=Mock(return_value=future))
                        for future in futures]
    node.create_client = Mock(side_effect=node.rpc_clients)
    node.destroy_client = Mock()
    node.restored = Mock(return_value=confirmed)
    node.spawn, node.call = object(), Mock()
    node.descriptions = {}

    def wait(predicate, timeout, reason):
        if not predicate():
            raise RuntimeError(reason)

    node.wait_for = wait
    return node


def test_fresh_odometry_confirms_reset_when_reply_is_lost(monkeypatch):
    future = Future()
    node = resetter(monkeypatch, [future], confirmed=True)
    node.restore_robot(ROBOT)
    assert future.cancelled()
    node.rpc_clients[0].call_async.assert_called_once()
    node.destroy_client.assert_called_once_with(node.rpc_clients[0])
    node.call.assert_not_called()


def test_unconfirmed_request_retries_until_acknowledged(monkeypatch):
    first, second = Future(), Future()
    node = resetter(monkeypatch, [first, second, completed(True)])
    node.restore_robot(ROBOT)
    assert first.cancelled() and second.cancelled()
    assert node.create_client.call_count == node.destroy_client.call_count == 3


def test_retry_bound_reports_the_robot_that_failed(monkeypatch):
    futures = [Future() for _ in range(3)]
    node = resetter(monkeypatch, futures)
    with pytest.raises(RuntimeError, match='robot_1'):
        node.restore_robot(ROBOT)
    assert all(future.cancelled() for future in futures)
    assert node.create_client.call_count == node.destroy_client.call_count == 3


def test_deleted_robot_respawns_from_the_complete_description(monkeypatch):
    node = resetter(monkeypatch, [completed(False)])
    xml = '<robot name="test"><link name="rear_dock_link"/></robot>'
    node.descriptions['robot_1'] = xml
    node.restore_robot(ROBOT)
    client, request, timeout = node.call.call_args.args
    assert client is node.spawn and timeout == 15.
    assert isinstance(request, SpawnEntity.Request)
    assert request.name == 'robot_1' and request.robot_namespace == '/robot_1'
    assert request.xml == xml


def test_failed_restore_keeps_motion_held(monkeypatch):
    node = resetter(monkeypatch, [])
    node.robots, node.lock, node.hold = [ROBOT], threading.Lock(), object()
    node.states = {'robot_1': dict(state='RESETTING', stamp=10.)}
    node.restore_robot = Mock(side_effect=RuntimeError('unconfirmed pose'))
    node.get_logger = Mock(return_value=SimpleNamespace(error=Mock()))
    response = node.reset(Trigger.Request(), Trigger.Response())
    assert not response.success and 'motion held' in response.message
    assert all(call.args[1].data is True for call in node.call.call_args_list)
    assert len(node.call.call_args_list) == 2 and not node.lock.locked()


def test_failed_physical_release_never_teleports_any_robot(monkeypatch):
    node = resetter(monkeypatch, [])
    node.robots, node.lock, node.hold = [ROBOT], threading.Lock(), object()
    node.detach_before_reset = True
    node.states = {'robot_1': dict(state='RESETTING', stamp=10.)}
    node.restore_robot = Mock()
    node.get_logger = Mock(return_value=SimpleNamespace(error=Mock()))

    node.release_docks = Mock(side_effect=RuntimeError('Physics release not confirmed'))
    response = node.reset(Trigger.Request(), Trigger.Response())
    assert not response.success and 'release' in response.message
    node.restore_robot.assert_not_called()


def test_fresh_physical_inventory_confirms_release_when_rpc_reply_is_lost(monkeypatch):
    future = Future()
    node = resetter(monkeypatch, [future])
    node.robots = [ROBOT]
    node.states = {'robot_1': dict(state='RESETTING', epoch='held')}
    node.dock_state = dict(epoch='held', stamp=10.2, physical=True, bonds=[])
    node.release_docks()
    assert future.cancelled()
    node.destroy_client.assert_called_once_with(node.rpc_clients[0])


def test_old_or_unverified_empty_inventory_cannot_authorize_teleport(monkeypatch):
    node = resetter(monkeypatch, [])
    node.robots = [ROBOT]
    node.states = {'robot_1': dict(state='RESETTING', epoch='held')}
    for state in [dict(epoch='previous', stamp=10.2, physical=True, bonds=[]),
                  dict(epoch='held', stamp=9., physical=True, bonds=[]),
                  dict(epoch='held', stamp=10.2, physical=False, bonds=[]),
                  dict(epoch='held', stamp=10.2, physical=True, bonds=[{}])]:
        node.dock_state = state
        assert not node.docks_released(10.)
