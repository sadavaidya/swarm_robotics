"""Parallel travel is allowed, while converging paths brake before collision."""
import math
from swarm_robot_shape_formation.concurrent import project, safe_command


def test_separated_robots_can_all_move_in_the_same_tick():
    poses = {str(i): [0., (i-4.5)*.6, 0.] for i in range(10)}
    commands = []
    for name, pose in poses.items():
        peers = {n: dict(pose=p, command=[.15, 0.]) for n, p in poses.items() if n != name}
        commands.append(safe_command(name, pose, .15, 0., peers, False))
    assert all(v == .15 for v, w in commands)


def test_head_on_approach_brakes():
    pose = [0., 0., 0.]
    peer = dict(pose=[.3, 0., math.pi], command=[.15, 0.])
    v, w = safe_command('a', pose, .15, 0., {'b': peer}, False)
    assert v == 0.


def test_turning_accounts_for_forward_axle():
    result = project([0., 0., 0.], 0., math.pi/2, 1.)
    assert abs(result[0]-.02) < 1e-9 and abs(result[1]+.02) < 1e-9


def test_offset_axle_can_escape_and_pass_a_neighbor_without_replanning_loop():
    from swarm_robot_behaviors.control import Pose2D
    from swarm_robot_shape_formation.shape_agent import ShapeAgent
    agent = ShapeAgent.__new__(ShapeAgent)
    agent.reconfiguring = False
    agent.route, agent.route_key, agent.replan_at = [], None, 0.
    agent.assembling, agent.clearance, agent.axle_offset = False, .3, .02
    agent.max_v, agent.max_w, agent.k_distance, agent.k_heading = .15, .6, .8, 2.
    agent.position_tolerance, agent.yaw_tolerance = .006, .025
    xyz = [1.1176947, 1.0683448, -3.0672184]
    goal = [-.881678, -1.213526, -.628319]
    peers = {'obstacle': dict(pose=[.8646181, 1.2296684, -1.4538107], command=[0., 0.])}
    for step in range(4000):
        if agent.at_target(xyz, goal):
            break
        v, w, phase = agent.navigation(Pose2D(*xyz), goal,
                                       [p['pose'][:2] for p in peers.values()], step*.05)
        v, w = safe_command('moving', xyz, v, w, peers, False)
        if phase == 'MOVING' and abs(v)+abs(w) < 1e-6:
            agent.route = []
        xyz = project(xyz, v, w, .05)
        assert math.dist(xyz[:2], peers['obstacle']['pose'][:2]) > .265
    assert agent.at_target(xyz, goal), xyz
