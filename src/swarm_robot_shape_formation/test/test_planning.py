"""Route safety and occupied-target handling are separate from role allocation."""
import math
from swarm_robot_shape_formation.planning import path_to, segment_clear


def test_astar_detours_around_a_robot_and_preserves_clearance():
    obstacles = [[0., 0.]]
    start, goal = [-1., 0.], [1., 0., 0.]
    route = path_to(start, goal, obstacles)
    assert route and len(route) > 1
    for point in route:
        assert segment_clear(start, point, obstacles, .35)
        start = point
    assert math.dist(route[-1], goal[:2]) < 1e-9


def test_occupied_goal_does_not_produce_a_route():
    assert path_to([-1., 0.], [0., 0., 0.], [[0., 0.]]) == []


def test_local_navigation_reuses_the_drive_controller_without_a_controller_node():
    from swarm_robot_behaviors.control import Pose2D
    from swarm_robot_shape_formation.shape_agent import ShapeAgent

    agent = ShapeAgent.__new__(ShapeAgent)
    agent.reconfiguring = False
    agent.route, agent.route_key, agent.replan_at = [], None, 0.
    agent.assembling = False
    agent.clearance, agent.axle_offset = .35, .02
    agent.max_v, agent.max_w = .15, .6
    agent.k_distance, agent.k_heading = .8, 2.
    v, w, phase = agent.navigation(Pose2D(-1., 0., 0.), [1., 0., 0.], [], 10.)
    assert phase == 'MOVING' and v == .15 and w == 0.


def test_axle_aware_navigation_reaches_pose_from_a_backward_heading():
    from swarm_robot_behaviors.control import Pose2D
    from swarm_robot_docking.protocol import wrap
    from swarm_robot_shape_formation.shape_agent import ShapeAgent

    agent = ShapeAgent.__new__(ShapeAgent)
    agent.reconfiguring = False
    agent.route, agent.route_key, agent.replan_at = [], None, 0.
    agent.assembling = False
    agent.clearance, agent.axle_offset = .35, .02
    agent.max_v, agent.max_w = .15, .6
    agent.k_distance, agent.k_heading = .8, 2.
    agent.position_tolerance, agent.yaw_tolerance = .025, .06
    target = [0., 0., math.pi/2]
    pose = Pose2D(-1., .2, math.pi)
    for tick in range(1500):
        if agent.at_target([pose.x, pose.y, pose.yaw], target):
            break
        v, w, _ = agent.navigation(pose, target, [], tick*.05)
        axle_x = pose.x+.02*math.cos(pose.yaw)+v*math.cos(pose.yaw)*.05
        axle_y = pose.y+.02*math.sin(pose.yaw)+v*math.sin(pose.yaw)*.05
        yaw = wrap(pose.yaw+w*.05)
        pose = Pose2D(axle_x-.02*math.cos(yaw), axle_y-.02*math.sin(yaw), yaw)
    assert agent.at_target([pose.x, pose.y, pose.yaw], target)
