# swarm_robot_dock_physics

[Workspace guide](../../README.md) · [Ten-robot formation guide](../swarm_robot_shape_formation/README.md)

This package supplies the **Gazebo Classic 11 world plugin** that physically
connects the robots in the ten-robot shape demo. It creates fixed joints between
nearby front/rear ports only after both robots consent, publishes the actual
joint inventory, and releases joints for shape changes or environment reset.
It is a simulation component, not a role allocator or robot motion controller.

## Files and dependencies

| File | Purpose |
| --- | --- |
| `src/dock_physics.cpp` | ROS consent/status interface and physics-thread joint creation/release. |
| `CMakeLists.txt` | Build/install `libswarm_dock_physics.so` and this README. |
| `package.xml` | ament_cmake, gazebo_dev, gazebo_ros, rclcpp, message/service and JSON dependencies; Gazebo plugin path export. |

The build needs the Classic development headers (`libgazebo-dev`,
`ros-humble-gazebo-dev`) and `nlohmann-json3-dev`. Use the workspace's rosdep
setup for the remaining dependencies.

## Build and load

```bash
source /opt/ros/humble/setup.bash
cd ~/robotics/projects/Swarm_robot
colcon build --symlink-install
source install/setup.bash
ros2 launch swarm_robot_shape_formation ten_robot_shape_demo.launch.py
```

There is no standalone `ros2 run` executable. The formation world's SDF loads:

```xml
<plugin name="dock_physics" filename="libswarm_dock_physics.so"/>
```

The world lives in `swarm_robot_shape_formation/worlds/formation_arena.world`.
Sourcing the built workspace supplies the installed plugin path. The plugin
discovers models whose canonical link is `<model_name>/base_footprint`; entity
names, frame prefixes and robot namespaces must match as in the supplied launch.

## Capture and release

Every robot publishes a complete consent snapshot on its own topic. A directed
edge `a:b` means **a's front port to b's rear port**. Capture requires:

- Both robots naming the same edge, current task epoch and assignment digest.
- Fresh simulation-time consent: less than 0.6 s old, with up to 0.25 s delivery-order allowance for a future stamp.
- Unoccupied ports and valid robot base links.
- Tip separation at most 0.018 m and relative planar heading at most 1.75 rad.
- Each body's linear speed at most 0.025 m/s and angular speed at most 0.15 rad/s.

Ports are evaluated at base-local `(±0.11, 0, 0.0535)` m, matching the shared
formation connector model. If that geometry changes, update both the Xacro and
plugin constants. Tips/stems are visual capture geometry; chassis/wheel contacts
remain physical. A successful latch fixes the **actual** relative pose. There
is no teleportation, attraction force or pull-in from distant poses.

ROS callbacks cache consent and release requests under a mutex. Joint creation,
detach and destruction happen in `WorldUpdateBegin`, on the physics thread.
Empty consent snapshots revoke pending requests. Once latched, a bond remains
until an explicit release/reset or simulation clock rewind.

On a shape change, `TRANSITION/HOLD` retains existing joints while the task node
waits for robot stop acknowledgments. `TRANSITION/RELEASE` releases them; fresh
status confirms an empty inventory before the new formation begins. `STOP`
preserves existing joints. The normal user interfaces are a new shape command
or `/swarm/reset`, which also coordinate robot controllers.

## ROS interfaces

JSON payloads use `std_msgs/msg/String`; the status schema has `version: 1`.

| Interface | Direction / purpose |
| --- | --- |
| `/swarm/shape_goal` | Retained reliable task/epoch and transition stage input. |
| `/robot_N/shape/dock_request` | Best-effort, depth-one consent snapshot input: `robot`, `epoch`, `digest`, `stamp`, `connections` with `a`/`b` edges. |
| `/swarm/dock_status` | Retained reliable status at about 20 Hz simulation time: `epoch`, `stamp`, `physical`, `bonds`, `pending`, `transition_stage`, `capture_distance`. |
| `/swarm/detach_all` (`std_srvs/srv/Trigger`) | Internal release request; success acknowledges physics-thread release. |

Each bond reports robot IDs, joint name and capture gap. Pending reasons include
`WAITING_FOR_BOTH_CONSENTS`, `ASSIGNMENT_MISMATCH`, `STALE_CONSENT`, and
`CAPTURE_GEOMETRY_OR_SETTLING`. A detach request times out after three wall
seconds if physics is not updating. Calling detach alone does not clear the
robots' active task and may permit later relatching; use the coordinated interfaces.

```bash
# After sourcing the workspace and matching the demo's DDS profile:
ros2 topic echo /swarm/dock_status --once --qos-durability transient_local
ros2 topic echo /robot_1/shape/dock_request --once --qos-reliability best_effort
```

## Verification and limits

The live physics verifier is maintained by the shape formation package because
it needs that world's robots and task protocol. Run only against a dedicated
simulation with explicit ROS/Gazebo isolation, as described in the workspace guide:

```bash
python3 src/swarm_robot_shape_formation/test/verify_latch_physics.py
```

It checks negative consent/capture cases, actual constraints after a pose
perturbation, and release. This plugin has no separate automatic unit test suite.
The shape package's tests cover consent and connected-completion decisions.

This is an ideal latch abstraction: no magnetic force curve, compliant dynamics,
latch actuator, electrical contact, breakaway force or hardware feasibility is
modeled. Captured angular freedom becomes a fixed constraint; joints do not
remain hinges. The current plugin uses shared `/swarm` task interfaces and
common-world ground truth; isolate separate experiments by ROS domain and Gazebo master.

## Original papers and software references

These references explain the platform and connected-module concept. The latch
implementation is custom; it does not reproduce a published hardware mechanism.

1. Nathan Koenig and Andrew Howard (2004), **Design and Use Paradigms for Gazebo,
   An Open-Source Multi-Robot Simulator**, IROS, pp. 2149–2154.
   [Original publication](https://robotics.usc.edu/publications/394/),
   [DOI: 10.1109/IROS.2004.1389727](https://doi.org/10.1109/IROS.2004.1389727).
   **Platform basis:** multi-robot physics simulation; the current code uses
   Classic 11 APIs rather than the paper's original software version.
2. Mark Yim et al. (2007), **Modular Self-Reconfigurable Robot Systems
   [Grand Challenges of Robotics]**, IEEE Robotics & Automation Magazine,
   14(1), pp. 43–52.
   [Author-hosted paper](https://cs.rice.edu/~mmoll/publications/yim2007modular-self-reconfigurable-robot-systems.pdf),
   [DOI: 10.1109/MRA.2007.339623](https://doi.org/10.1109/MRA.2007.339623).
   **Conceptual background:** docking interfaces and changing connectivity;
   this overview is not a connector design or reconfiguration algorithm implemented here.

Runtime cross-model joints are also demonstrated by
[PAL Robotics' gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher).
That is a software reference, not a paper or a dependency of this package.

Licensed under the [MIT License](../../LICENSE).
