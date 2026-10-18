# Five robots following a leader

[Workspace guide](../../README.md)

ROS 2 Humble / Gazebo Classic 11 experiment: one leader and four followers in a
10 m x 10 m clear arena, bounded by primitive 0.4 m high walls. The existing
`swarm_robot_description` package, Xacro, drive plugin, sensors and single-robot
launch files are reused without modification. No meshes or model downloads.

## Build, source and launch

```bash
source /opt/ros/humble/setup.bash
cd ~/robotics/projects/swarm_robotics
colcon build --symlink-install
source install/setup.bash

# Stop any previous simulation using the same Gazebo master before this command.
env -u GTK_PATH -u GIO_MODULE_DIR \
  ros2 launch swarm_robot_behaviors five_robot_leader_follow.launch.py
```

This starts Gazebo, all five robots, their state publishers, the common-world
TF links, five controllers and RViz. The GTK/GIO workaround is for the inherited
VS Code Snap environment on this machine; a regular terminal can launch directly.

```bash
# Headless experiment (behaviors still start automatically)
ros2 launch swarm_robot_behaviors five_robot_leader_follow.launch.py gui:=false rviz:=false

# Spawn and visualize, but hold all controllers stopped initially
ros2 launch swarm_robot_behaviors five_robot_leader_follow.launch.py autostart:=false
```

To run alongside a separate single-robot session, isolate BOTH ROS and Gazebo.
Use these exports in every terminal interacting with the isolated experiment:

```bash
export ROS_DOMAIN_ID=73
export GAZEBO_MASTER_URI=http://127.0.0.1:11373
```

Validation used that isolation and did not stop the pre-existing single-robot
session. All controllers, state publishers and RViz use `/clock` simulation time.

## Files and architecture

| File | Responsibility |
| --- | --- |
| `worlds/swarm_arena.world` | Flat ground and four walls; interior extends from -5 to +5 m |
| `config/experiment.yaml` | Spawn positions, chain targets, gains, limits and leader route |
| `launch/five_robot_leader_follow.launch.py` | One server; reuse existing spawn launch five times; controllers and RViz |
| `rviz/five_robots.rviz` | All models and distinct colored odometry arrows in `world`, plus TF |
| `swarm_robot_behaviors/leader_controller.py` | Loop through a rectangular list of world waypoints |
| `swarm_robot_behaviors/follower_controller.py` | Reusable proportional target-following node |
| `swarm_robot_behaviors/pose_provider.py` | Fresh stamped odometry transformed into the common world frame |
| `swarm_robot_behaviors/controller_base.py` | Command timer, range safety, stale-data stops and enable service |
| `swarm_robot_behaviors/control.py` | Pure planar control and safety logic |
| `test/test_control.py` | Control signs, wrapping, bounds, deadbands and safety regressions |
| `test/verify_live.py` | Opt-in full-experiment and physical obstacle checks |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | ROS Python package installation |

```text
robot_1 (leader) <- robot_2 <- robot_3 <- robot_4 <- robot_5

Gazebo world-pose odometry --TF--> PoseProvider --> leader/follower P controller
Own three-ray front scan -----------------------> safety override
                                                  |
                                        /robot_N/cmd_vel
                                                  |
                                     existing Gazebo drive plugin
```

Each robot starts facing +X:

| Robot | x | y | Target |
| --- | ---: | ---: | --- |
| robot_1 | -2.0 | 0.0 | World waypoints |
| robot_2 | -3.0 | -0.8 | robot_1 |
| robot_3 | -3.0 | 0.8 | robot_2 |
| robot_4 | -4.0 | -1.5 | robot_3 |
| robot_5 | -4.0 | 1.5 | robot_4 |

Wait roughly 30–60 simulation seconds for the initially scattered robots to
settle into a chain. The leader cruises at 0.12 m/s, allowing followers limited
to 0.20 m/s to catch up. The loop's waypoints are (2.5, 0), (2.5, 2.5),
(-2, 2.5), (-2, 0), well inside the boundary. It turns in place for large heading
changes and resumes forward motion when aligned. The full circuit takes about
two minutes of simulation time, including turns.

## How following and safety work

For each follower, common-world positions give distance and bearing relative to
its own current heading:

```text
distance_error = distance_to_target - desired_follow_distance
angular_z = clamp(k_heading * wrapped_bearing_error, ±max_angular_speed)
linear_x = clamp(k_distance * distance_error, 0, max_linear_speed)
```

Forward speed is additionally multiplied by `max(0, cos(bearing_error))` and
set to zero if the absolute bearing exceeds `turn_in_place_angle`. Followers
never reverse blindly. Distance and heading deadbands reduce stationary jitter.

This is pure pursuit of the preceding robot's current position, not a replay of
its path. **A pure P controller needs a positive spacing error to keep moving.**
With 0.12 m/s cruise and k_distance=0.6, straight-line steady spacing is roughly
`0.6 + 0.12 / 0.6 = 0.8 m`. When the target stops, spacing approaches the 0.6 m
setpoint plus deadband. Turns and startup change spacing temporarily.

Every robot's own `/range/front` is the existing three-ray LaserScan, not a
full lidar. Its minimum valid return supplies the range guard. `+inf` means
clear/no return, `-inf` means too close, and empty/all-invalid scans force a stop.
Below 0.25 m, the guard overrides following/waypoints with zero forward speed
and a 0.5 rad/s left turn. It stays latched for at least 0.7 s and until the
reading exceeds 0.35 m. The fixed left choice is a simple escape heuristic;
three near-forward rays cannot reliably determine which side is safer.

No command is generated from missing/stale own pose or range data. Missing or
stale target pose stops normal follower motion (the range escape can still turn
in place). Default freshness timeout is 0.5 simulation seconds. Controller
shutdown attempts to publish zero; a hard kill or hung process cannot do so,
and the retained Gazebo drive plugin has no cmd_vel watchdog.

## Privileged pose information and TF

**SIMULATION GROUND TRUTH:** the existing model explicitly sets
`odometry_source=1`, so its odometry contains Gazebo world pose, including its
nonzero spawn offset. This is confirmed by the
[Gazebo drive implementation](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp).
The experiment publishes identity static transforms:

```text
world
├── robot_1/odom -> robot_1/base_footprint -> robot_1/base_link -> robot_1/... links
├── robot_2/odom -> robot_2/base_footprint -> robot_2/base_link -> robot_2/... links
├── robot_3/odom -> robot_3/base_footprint -> robot_3/base_link -> robot_3/... links
├── robot_4/odom -> robot_4/base_footprint -> robot_4/base_link -> robot_4/... links
└── robot_5/odom -> robot_5/base_footprint -> robot_5/base_link -> robot_5/... links
```

The helper uses TF to transform stamped odometry into `world`, checks frame IDs,
message age and pose validity, then supplies a planar pose. It does not simply
subtract coordinates labeled with unrelated frames. Spawn offsets must NOT be
added again in world-to-odom transforms. If the model later switches to encoder
odometry, these identity transforms are no longer generally valid and this
provider must be replaced by a realistic relative-localization interface.

All model links/joints and TF IDs have unique prefixes. `/tf`, `/tf_static` and
`/clock` are shared; robot topics are separated by namespaces. The drive plugin
owns odom-to-base TF, robot_state_publisher owns the robot's internal tree, and
one static broadcaster per robot owns world-to-odom.

## Inspect and control the experiment

Source the environment in each terminal (and use the same isolation exports if
applicable):

```bash
source /opt/ros/humble/setup.bash
source ~/robotics/projects/swarm_robotics/install/setup.bash
ros2 topic list | grep robot_

ros2 topic echo /robot_1/cmd_vel --once
ros2 topic echo /robot_3/odom --once
ros2 topic echo /robot_4/imu/data --once
ros2 topic echo /robot_5/range/front --once
ros2 topic echo /robot_2/joint_states --once
ros2 run tf2_ros tf2_echo world robot_5/base_link
ros2 topic echo /robot_3/behavior/state --once
ros2 topic echo /robot_3/behavior/error --once
```

Each robot has cmd_vel, odom, imu/data, range/front, joint_states and
robot_description. `behavior/state` reports waiting, following/waypoint,
avoiding or disabled. Follower `behavior/error` is a diagnostic Vector3Stamped:
x = distance in metres, y = bearing error in radians, z = spacing error in metres
(these are scalar diagnostics, not a geometric vector to transform).

Pause all behaviors with their services. These publish zero continuously while
disabled, so there is no competing manual cmd_vel publisher:

```bash
ros2 service call /robot_1/leader_controller/set_enabled std_srvs/srv/SetBool '{data: false}'
for i in 2 3 4 5; do
  ros2 service call /robot_${i}/follower_controller/set_enabled std_srvs/srv/SetBool '{data: false}'
done
```

Use `data: true` to re-enable them. To stop the whole experiment, Ctrl+C the
launch terminal. Do not run a teleop publisher against an enabled controller.
RViz uses `world`, shows all five models, and gives each odometry trail a
different color. Use Gazebo's model list to select/follow a tiny 100 mm robot
when the full-arena camera makes it hard to see. Toggle TF names in RViz when
inspecting individual frames.

## Tuning

Edit `config/experiment.yaml` and relaunch, or pass a separate file with
`config:=/absolute/path/experiment.yaml`. Parameters are read-only after startup;
use the enable service for pausing/resuming. The supplied RViz view assumes the
five default robot namespaces and the `world` frame.

| Parameter | Default | Effect |
| --- | ---: | --- |
| follower.desired_follow_distance | 0.6 m | Spacing setpoint |
| common.k_distance | 0.6 | Translation responsiveness; higher reduces moving spacing error |
| common.k_heading | 1.5 | Turning responsiveness |
| common.max_linear_speed | 0.20 m/s | Maximum follower speed; also caps leader cruise |
| leader.cruise_speed | 0.12 m/s | Leader speed; leave catch-up margin for followers |
| common.max_angular_speed | 0.8 rad/s | Follower turning limit |
| leader.max_angular_speed | 0.7 rad/s | Leader turning limit |
| common.distance_deadband | 0.03 m | Forward-motion deadband near spacing setpoint |
| common.heading_deadband | 0.03 rad | Heading jitter deadband |
| common.turn_in_place_angle | 0.8 rad | Stop translation while facing far away from target |
| common.stop_distance / clear_distance | 0.25 / 0.35 m | Obstacle trigger / clearance hysteresis |
| common.avoid_turn_speed / min_turn_time | 0.5 rad/s / 0.7 s | Emergency-turn behavior |
| leader.waypoints / waypoint_tolerance | listed above / 0.20 m | Route and corner acceptance |

## Limitations and relationship to the other demos

- Pose access is privileged world ground truth; no realistic localization or
  sensing of the target robot is implemented.
- A front fan is not comprehensive collision avoidance. The current ray height
  is about 71 mm, above the 66 mm chassis top of another robot: it can miss
  robots and low objects, as well as side/rear obstacles. The existing model was
  preserved. Target-distance control supplies spacing, not a collision guarantee.
- Followers can cut corners, paths can cross, and a simple left-turn escape can
  oscillate or get stuck. No planner, formation optimizer or guaranteed spacing.
- There is no independent command watchdog after a controller hard failure.
- DDS represents communications; no Wi-Fi propagation, latency or loss model.
- No Nav2, SLAM, mapping, docking, learning, consensus or other swarm algorithms.

This package remains a simple leader/follower baseline. The sibling
[shape formation package](../swarm_robot_shape_formation/README.md) adds independent
velocity watchdogs, distributed role allocation and physical docking for its
own demo; those features are not enabled by this leader/follower launch. Real
relative sensing, controlled communication loss and hardware validation remain
future work across the workspace.

## Verification

```bash
colcon test
colcon test-result --verbose
```

For the live check, start a fresh dedicated experiment and then run:

```bash
python3 src/swarm_robot_behaviors/test/verify_live.py --seconds 160 --safety
```

This opt-in test observes autonomous motion, checks all robot topics/frame IDs,
common-world TF, command limits, path progress and chain spacing. It then
disables controllers, inserts a temporary wall about 0.18 m ahead of each
sensor in turn, enables only that robot, and checks zero forward command plus
actual physical rotation. It deletes its obstacles and leaves controllers
disabled. It does not run automatically in colcon test.

Validated on this machine on 2026-10-05, on ROS domain 73 and Gazebo port 11373:

- Both packages built; existing description checks and all nine behavior unit
  tests passed (`colcon test-result`: 19 reported tests, zero failures/errors).
  `ament_flake8` passed for the behavior package.
- All five copies spawned from the unchanged description. All 25 requested
  namespaced command/odometry/IMU/range/joint topics supplied data. All 40 robot
  TF frames resolved from world with unique prefixes and consistent parents;
  each robot had one command publisher.
- During 160 simulation seconds, the leader traversed every waypoint and
  traveled 18.38 m. The four followers traveled 16.76, 15.42, 14.76 and 13.61 m.
- All four gaps converged to approximately 0.80 m by 30 seconds. Mean gaps after
  settling were 0.769, 0.754, 0.740 and 0.725 m. Minimum observed pairwise centre
  separation was 0.639 m, sampled at 0.5 s intervals; this is not a general
  collision-free guarantee.
- Each robot independently detected an actual temporary wall at approximately
  0.180 m, commanded zero forward velocity, and physically turned 0.38–0.46 rad
  during the observation interval. After obstacle removal, each exited the
  avoidance state. Temporary obstacles were removed and controllers disabled.
- Gazebo GUI and the common-world RViz configuration started against the
  isolated server. The existing single-robot package remained byte-for-byte
  unchanged and its already-running simulation was preserved.

Raw live results are saved in `log/five_robot_validation/` at workspace level.

## Original papers and conceptual references

The controllers here are custom proportional distance/bearing controllers with
a front-range stop/turn override. The references below provide formation and
middleware context; they do not imply an implementation of a paper's full controller.

1. Jaydev P. Desai, James P. Ostrowski and Vijay Kumar (2001), **Modeling and
   Control of Formations of Nonholonomic Mobile Robots**, IEEE Transactions on
   Robotics and Automation, 17(6), pp. 905–908.
   [Author's publication record](https://robomed.gatech.edu/publications/),
   [DOI: 10.1109/70.976023](https://doi.org/10.1109/70.976023).
   **Conceptual background:** formation relationships and control graphs for
   nonholonomic robots. Our fixed leader/follower chain does not implement the
   paper's nonlinear controllers or graph-switching framework.
2. Steve Macenski, Tully Foote, Brian Gerkey, Chris Lalancette and William Woodall
   (2022), **Robot Operating System 2: Design, architecture, and uses in the wild**,
   Science Robotics, 7(66), eabm6074.
   [Author manuscript](https://arxiv.org/abs/2211.07752),
   [DOI: 10.1126/scirobotics.abm6074](https://doi.org/10.1126/scirobotics.abm6074).
   **Platform basis:** reusable namespaced ROS 2 controllers and DDS message exchange.

Licensed under the [MIT License](../../LICENSE).
