# Minimal swarm-capable robot

[Workspace guide](../../README.md)

One functional robot for ROS 2 Humble / Ubuntu 22.04 / Gazebo Classic 11.
Primitive geometry only; the existing chassis, wheel contacts, passive caster,
and working Gazebo differential drive are retained. The shared model now also
includes a passive front docking plate/frame; docking behavior is launched
separately by swarm_robot_docking. There is no CAD redesign or mesh dependency.

## Files

| File | Purpose |
| --- | --- |
| `urdf/swarm_robot.urdf.xacro` | Dimensions, mass, primitive geometry, joints, namespace/prefix arguments |
| `urdf/swarm_robot.gazebo.xacro` | Existing drive and wheel joint-state plugins, now namespace-ready |
| `urdf/sensors.xacro` | Central IMU and small front-facing ray sensor |
| `urdf/docking.xacro` | Passive front/optional rear plates, or ideal two-port formation connectors and prefixed mating frames |
| `launch/sim.launch.py` | Start one Gazebo server, spawn one robot, optionally start RViz |
| `launch/spawn.launch.py` | Reusable single-robot spawn and robot_state_publisher for an existing server |
| `launch/rviz.launch.py` | Adapt topics and frames to the chosen namespace in a temporary RViz config |
| `rviz/swarm_robot.rviz` | Robot, TF, odometry, IMU/range axes and range returns |
| `worlds/empty.world` | Existing ground plane and 1 ms physics steps, unchanged |
| `worlds/range_target.sdf` | Optional primitive obstacle for checking distance measurements |
| `package.xml` | Runtime dependencies, including YAML for the RViz configuration |

`CMakeLists.txt` already installs these asset directories and requires no change.

## Mechanics and hardware abstraction

The chassis is 100 x 100 x 25 mm, with a 0.20 kg body. Body mass represents the
combined structure, battery and electronics; separate ESP32, battery and motor
driver boxes would add no useful behavior. Wheels have a 20 mm radius, 10 mm
width, 15 g mass and 90 mm centre separation. The axle sits 20 mm forward of the
body centre. The rear support is an 8 mm radius, 5 g sphere at x = -35 mm with
zero tangential friction, approximating a passive ball caster. The centre of
mass lies inside the support triangle. Total modeled mass with sensors and the passive docking plate is 240 g.

The chassis underside is 41 mm above the ground, leaving 1 mm above the wheel
tops. Standard box/cylinder/sphere inertias are retained. The IMU is near the
centre of the top face; the range sensor is above the front lip, leaving the
lower front available for future docking. Sensor markers are 1 g each with no
extra collision geometry. +X is forward, +Y left, +Z up.

```text
ROS / future swarm software
          |
    cmd_vel (Twist)
          |
Gazebo differential-drive plugin
          |
 left/right wheel joints
          |
  joint_states + odom
```

`libgazebo_ros_diff_drive.so` is the existing low-level simulated controller.
It limits wheel torque to 0.05 Nm and wheel linear acceleration to 0.5 m/s².
No controller manager or ros2_control controller YAML is needed for this
plugin-based setup. Do not also attach a second drive controller to the wheels.
Future ESP32-S3 firmware and TB6612 hardware replace this low-level actuation
interface; N20 encoders supply the feedback currently obtained from Gazebo.
Motor electrical behavior, battery discharge and encoder pulse quantization
are not modeled.

**ROS 2 DDS is the communication equivalent of inter-robot Wi-Fi here.** It
provides software message exchange, without modeling RF range, attenuation,
interference, packet loss, or ESP32 networking.

## Build and launch

```bash
source /opt/ros/humble/setup.bash
cd ~/robotics/projects/Swarm_robot
colcon build --symlink-install
source install/setup.bash
ros2 launch swarm_robot_description sim.launch.py rviz:=true
```

For a server without windows:

```bash
ros2 launch swarm_robot_description sim.launch.py gui:=false rviz:=false
```

To open RViz for an already running robot:

```bash
ros2 launch swarm_robot_description rviz.launch.py
```

On this machine, a VS Code Snap terminal can inject incompatible GTK/GIO paths.
The following process-local workaround was used successfully during validation:

```bash
env -u GTK_PATH -u GIO_MODULE_DIR ros2 launch swarm_robot_description sim.launch.py rviz:=true
```

All robot state, visualization and spawn nodes use simulated time; Gazebo
publishes `/clock`. Sensor/plugin stamps use Gazebo simulation time. If setting
up another Humble machine, install declared dependencies with:

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## Drive and inspect

Source each additional terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/robotics/projects/Swarm_robot/install/setup.bash
```

Run one motion publisher at a time. End it with Ctrl+C before starting the next.
**Ctrl+C only ends the publisher; the drive plugin retains its last command.**
Send the explicit zero command to stop. There is no command watchdog in this
retained plugin setup.

```bash
# Forward, 0.05 m/s
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}'

# Backward, 0.05 m/s
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.05}, angular: {z: 0.0}}'

# Rotate left, 0.5 rad/s (use -0.5 to rotate right)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.5}}'

# Stop after ending the active motion publisher
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'

ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once --qos-reliability best_effort
ros2 topic echo /range/front --once --qos-reliability best_effort
ros2 topic echo /joint_states --once

# Wheel angles (radians) and angular velocities (radians/second)
ros2 topic echo /joint_states --field position
ros2 topic echo /joint_states --field velocity

ros2 run tf2_ros tf2_echo odom imu_link
ros2 run tf2_ros tf2_echo odom front_range_link
```

## Topics and sensor interpretation

| Default topic | Message type | Nominal rate / purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Input: linear.x and angular.z |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz; Gazebo world-pose odometry |
| `/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz; actual simulated wheel positions and velocities |
| `/imu/data` | `sensor_msgs/msg/Imu` | 100 Hz; orientation, angular velocity, linear acceleration |
| `/range/front` | `sensor_msgs/msg/LaserScan` | 20 Hz; only three rays across 0.05 rad (about 2.9 degrees) |
| `/robot_description` | `std_msgs/msg/String` | Transient-local robot description |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Shared transforms with unique frame IDs |
| `/clock` | `rosgraph_msgs/msg/Clock` | Shared simulation time |

Rates are per simulation second. Wall-clock rates depend on Gazebo performance.

**Encoder abstraction:** the joint-state plugin reads the physics engine's wheel
joint angles and velocities. Arrays are ordered according to `name` (normally
left then right). At 0.05 m/s, both wheels turn near 2.5 rad/s. Wheel travel is
`radius * delta_angle`; ideal differential kinematics give
`v = radius * (right_velocity + left_velocity) / 2` and
`yaw_rate = radius * (right_velocity - left_velocity) / wheel_separation`.
There are no fabricated tick counts, encoder noise, or effort readings.

**Odometry remains explicitly world-based (`odometry_source=1`)**, preserving
the previously working behavior and the body-centred base_footprint with its
forward-offset axle. It is ground truth, not integrated encoder dead reckoning
or an IMU fusion estimate. Wheel feedback still responds to physical wheel
motion and can differ from ground travel under slip. The published covariance
is the plugin's fixed default, not a calibrated uncertainty estimate.

The IMU frame is aligned to the chassis. Orientation is a quaternion referenced
to Gazebo world, angular velocity is in rad/s, and acceleration is in m/s².
A level stationary IMU reads roughly +9.81 on Z (specific force including the
gravity response). No noise, bias or drift is configured; zero covariances are
not a calibrated real sensor model. RViz displays the IMU frame axes; it does
not require an additional IMU visualization plugin.

**The front sensor is a tiny ray scanner, not a full lidar.** Its valid range is
0.02–2.0 m. `ranges[1]` points directly forward; `min(ranges)` is the closest of
its three returns. `+inf` means no obstacle within range on that ray. Objects
must intersect the ray height (about 71 mm). An empty world normally gives
three infinite returns and no visible scan points in RViz.

LaserScan output is deliberate: the installed Classic ray plugin's Range
conversion emits FLT_MAX when all rays miss, and this Humble Python Range
binding rejects that value. LaserScan preserves infinities and works with
`ros2 topic echo` without patching system packages or adding a relay node.

For a visible distance target, start a fresh default simulation at the origin
and run this before driving:

```bash
ros2 run gazebo_ros spawn_entity.py -entity range_target \
  -file "$(ros2 pkg prefix --share swarm_robot_description)/worlds/range_target.sdf" \
  -x 0.65 -z 0.1
ros2 topic echo /range/front --once
# Centre ray should read about 0.548 m: 0.65 - 0.05 - 0.052.
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'range_target'}"
```

## TF and namespace preparation

```text
odom                         (drive plugin)
└── base_footprint
    └── base_link            (robot_state_publisher)
        ├── left_wheel_link
        ├── right_wheel_link
        ├── caster_link
        ├── front_dock_link
        │   └── front_dock_frame
        ├── imu_link
        └── front_range_link
```

The drive plugin publishes only odom -> base_footprint. robot_state_publisher
owns all internal transforms, using joint_states for the wheels. Wheel TF is
disabled in the drive plugin to avoid duplicate publishers. The old wheel and
caster link names now use the consistent `_link` suffix.

To run this same single robot with a namespace, stop the previous simulation:

```bash
ros2 launch swarm_robot_description sim.launch.py namespace:=robot_1 rviz:=true
ros2 topic echo /robot_1/imu/data --once
ros2 topic echo /robot_1/range/front --once
ros2 run tf2_ros tf2_echo robot_1/odom robot_1/imu_link
# To open its RViz separately:
ros2 launch swarm_robot_description rviz.launch.py namespace:=robot_1
```

`namespace` defaults to empty, retaining the requested root topics. With
`namespace:=robot_1`, every robot topic becomes `/robot_1/...`, the Gazebo entity
becomes `robot_1`, and every link, joint and TF frame gets `robot_1/` prepended.
`frame_prefix:=auto` derives this prefix; explicit overrides must match between
spawn and RViz. Prefixes are embedded once in the URDF; RSP's own frame_prefix
parameter is intentionally unused. `/tf`, `/tf_static`, `/clock` and Gazebo
spawn/delete services remain shared. DDS namespaces do not themselves prefix
TF frame IDs, hence the separate Xacro prefix.

Launch arguments also include `entity_name`, `x`, `y`, `z`, `yaw`, and (on sim)
`world`, `gui`, `rviz`. Direct Xacro users can pass `robot_namespace:=/robot_1`
and `frame_prefix:=robot_1/` explicitly.

Multi-robot launches start Gazebo once and reuse `spawn.launch.py` with distinct
namespaces/entities, prefixes and non-overlapping poses. For one spawn into an existing server:

```bash
ros2 launch swarm_robot_description spawn.launch.py namespace:=robot_1 x:=0.5 y:=0.5
```

Do not launch `sim.launch.py` repeatedly against the same Gazebo master. This
package supplies the shared robot assets; multi-robot orchestration is provided
by the sibling behavior, pairing and shape formation packages. This package's
standalone spawn does not add a common `world` parent. The multi-robot demos add
identity `world -> robot_N/odom` transforms because odometry uses Gazebo world
coordinates; this is not a localization estimate.

## Verification

Build and existing package lint checks:

```bash
colcon test --packages-select swarm_robot_description
colcon test-result --verbose
```

Validated on this machine on 2026-10-05:

- `colcon build --symlink-install` succeeded; all four lint checks passed
  (`colcon test-result`: 10 reported tests, zero errors/failures).
- Xacro expansion, `check_urdf` and Gazebo SDF conversion passed for empty and
  `robot_1/` prefixes; caster contact friction remains zero after fixed-joint
  reduction, and both sensor poses are preserved.
- Gazebo Classic and RViz started. Both default headless-server and namespaced
  GUI launches spawned successfully. RViz subscribes to the chosen namespace.
- All four requested topic echo commands received real messages. Measured
  simulation-time rates: odom/joint_states 50 Hz, IMU 100 Hz, front scanner 20 Hz.
- Default-run two-second motion checks: forward +0.0981 m, backward -0.0979 m,
  left +0.9356 rad, right -0.9769 rad. Wheel feedback was about ±2.5 rad/s during
  straight driving; the IMU measured about ±0.5 rad/s during turns.
- Stopped position drift was about 0.045 mm over one simulation second.
- A temporary obstacle expected at 0.548 m measured 0.54796 m; removing it
  restored infinite no-return readings. The temporary obstacle was deleted.
- All seven robot frames resolved from odom. The same sensor, TF, range and
  motion checks passed with `namespace:=robot_1 x:=0.3 y:=0.2 yaw:=0.4`,
  with no root-level robot topics and matching prefixed message frames.

These are ideal sensor and ground-truth odometry checks, not hardware accuracy
or swarm coordination validation. The robot rotates around the forward axle,
so its body-centred base_footprint traces a small arc when turning.

Plugin references: [drive](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp),
[joint feedback](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp),
[IMU](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp),
[ray sensor](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp).


## Passive docking model extension (2026-10-07)

The existing single-robot and leader-following launch files still use the same
shared Xacro. It now includes a 36 x 12 x 14 mm, 3 g orange front plate fixed at
x=56 mm and a front_dock_frame on its outer face at x=62 mm. All names inherit
the existing frame prefix. This model change adds no actuation or attachment.
The separate swarm_robot_docking launch implements PAIR auctions and logical
mating; see that package's README for commands and current verification.
The validation results above describe the original single-robot milestone.

## Optional rear and formation connectors

The spawn launch and Xacro support `rear_docking` and `formation_docking`, both
defaulting to `false`. `rear_docking:=true` adds a rear plate/frame for the
passive model. `formation_docking:=true` selects ideal front/rear capture stems
and tips instead of the original front plate; the ten-robot formation launch
enables both options. No CAD meshes are used.

```bash
# Spawn a two-port model into an existing server; this alone does not enable latching.
ros2 launch swarm_robot_description spawn.launch.py namespace:=robot_1 \
  rear_docking:=true formation_docking:=true x:=0.5 y:=0.5
```

Front/rear capture frames sit at ±110 mm from the body center. Their collision-free
visual stems/tips represent ideal angular capture. Actual runtime joints require
the [physics plugin](../swarm_robot_dock_physics/README.md) and bilateral consent
from the [formation agents](../swarm_robot_shape_formation/README.md). The original
single-robot and five-robot launches retain passive docking geometry.

## Original papers and platform references

This package implements a custom primitive robot model. These are citations for
the software foundations, not papers specifying this robot's hardware or dimensions.

1. Steve Macenski, Tully Foote, Brian Gerkey, Chris Lalancette and William Woodall
   (2022), **Robot Operating System 2: Design, architecture, and uses in the wild**,
   Science Robotics, 7(66), eabm6074.
   [Author manuscript](https://arxiv.org/abs/2211.07752),
   [DOI: 10.1126/scirobotics.abm6074](https://doi.org/10.1126/scirobotics.abm6074).
   **Platform basis:** ROS 2 nodes, message interfaces, namespaces and DDS communication.
2. Nathan Koenig and Andrew Howard (2004), **Design and Use Paradigms for Gazebo,
   An Open-Source Multi-Robot Simulator**, IROS, pp. 2149–2154.
   [Original publication](https://robotics.usc.edu/publications/394/),
   [DOI: 10.1109/IROS.2004.1389727](https://doi.org/10.1109/IROS.2004.1389727).
   **Platform basis:** robot physics and sensor simulation; the runtime here is
   Classic 11, not the original 2004 release.

The plugin source links above document the concrete drive, encoder feedback and
sensor behavior used here.

Licensed under the [MIT License](../../LICENSE).
