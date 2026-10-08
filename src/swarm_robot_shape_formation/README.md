# Ten robots forming physically connected planar shapes

[Workspace guide](../../README.md) · [Physical latch plugin](../swarm_robot_dock_physics/README.md)

This package runs ten primitive differential-drive robots in Gazebo Classic 11
with ROS 2 Humble. Robots exchange bids to choose roles, travel concurrently,
and connect front-to-rear to build LINE, CIRCLE or an approximate SQUARE.
Completion requires actual Gazebo joints connecting the entire requested graph.
Merely standing near the target points does not count as completion.

## Build and run

Stop an earlier swarm launch before starting this one on the same ROS domain.

```bash
cd ~/robotics/projects/Swarm_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
env -u GTK_PATH -u GIO_MODULE_DIR ros2 launch swarm_robot_shape_formation ten_robot_shape_demo.launch.py
```

For a repeatable random layout add `spawn_seed:=0`. Default `spawn_seed:=-1`
chooses a fresh seed and prints it with the spawn positions. Ten robots start
at least 0.8 m apart inside the arena. `gui:=false rviz:=false` runs headless.
`random_spawn:=false` uses the configured outer-ring layout.

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/robotics/projects/Swarm_robot/install/setup.bash
shape_share=$(ros2 pkg prefix --share swarm_robot_shape_formation)
export FASTRTPS_DEFAULT_PROFILES_FILE="$shape_share/config/fastdds_udp.xml"
ros2 topic pub --once --keep-alive 2 --qos-durability volatile \
  /swarm/shape_command std_msgs/msg/String "{data: 'LINE'}"
```

Use `CIRCLE` or `SQUARE` instead of `LINE`. The command is case-insensitive.
The launch uses a UDP DDS profile; use the same profile for CLI tools.

```bash
# Progress, robot states, concurrent movers and verified physical connections
ros2 topic echo /swarm/formation_status std_msgs/msg/String --qos-durability transient_local
# The physics plugin's actual joint inventory
ros2 topic echo /swarm/dock_status std_msgs/msg/String --qos-durability transient_local
# Stop motion; existing physical connections remain latched
ros2 topic pub --once --keep-alive 2 /swarm/shape_command std_msgs/msg/String "{data: 'STOP'}"
# Release all connections, restore this run's original spawn poses, clear the task
ros2 service call /swarm/reset std_srvs/srv/Trigger '{}'
```

**Request a different shape on the same topic to change the current formation.**
The task publisher first holds every robot and confirms zero drive commands,
then releases the physical latches and waits for fresh empty joint inventory.
Robots negotiate new roles and form the requested shape from their current
positions. This does not restore spawn poses or call Gazebo's teleport service.
For example, after CIRCLE completes:

```bash
ros2 topic pub --once --keep-alive 2 /swarm/shape_command std_msgs/msg/String "{data: 'SQUARE'}"
# Once SQUARE completes, request LINE the same way
ros2 topic pub --once --keep-alive 2 /swarm/shape_command std_msgs/msg/String "{data: 'LINE'}"
```

During a transition the monitor reports `STOPPING_FOR_SHAPE_CHANGE`, then
`DISCONNECTING`. A new valid request replaces the pending shape; the latest
request wins. Repeating the active shape leaves the current task intact.
`STOP` cancels a pending transition and holds the robots wherever they are.
If stopping/release cannot be confirmed within 30 wall seconds, the fleet
remains held with `TRANSITION_FAILED`; resend a shape to retry. Invalid shapes
leave the current task intact. `/swarm/reset` cancels a pending transition.

Reset remains an explicit arena reset. It releases latches before teleporting robots,
confirms fresh odometry at the restored poses, and never rewinds `/clock`.
A failed reset keeps the fleet stopped; its service reply identifies the failure.
Reset restores the same sampled layout; relaunch for a new random layout.

## Assembly and control

1. `shape_goal` broadcasts anonymous target poses and separated approach poses.
   It does not assign robot IDs, collect bids or publish wheel commands.
2. Identical `shape_agent` nodes run a single-role distributed bid/max-consensus
   auction. Fixed distance/heading bids and deterministic ties produce one robot
   per role. All ten peers must agree on the assignment before moving.
3. Robots independently follow A* paths to their approach poses **concurrently**.
   A peer barrier ensures the approach positions are reached before assembly.
4. Robots approach the connected template concurrently at reduced speed. Local
   prediction slows/brakes for conflicts. Local waits and alignment pauses are
   expected; there is no global token authorizing only one robot to move.
   After a shape change, reduced clearance and speed let robots safely leave
   the previously packed assembly before using normal transit clearance.
5. Neighboring robots independently consent to latch after both reach their
   docking poses. The physics plugin checks current port separation, angle and
   speed, then creates a joint at their **actual** relative pose. It does not
   teleport robots or pull distant robots into position.
6. A robot with a physical latch stops driving. The monitor declares COMPLETE
   only after the expected graph exists and the relaxed shape bounds hold.

Pose targets guide approach; the goal remains a connected, approximate shape.
Completion permits up to 5 cm positional and 0.25 rad heading error per role.
Local approach is tighter so connector capture can succeed. LINE has nine
front-to-rear bonds. CIRCLE and SQUARE have ten, including the closing bond.
All ten robots stop when connected.

The auction is decentralized; shared templates, a common world frame, known
membership and all-to-all peer communication are still assumptions. This is
not anonymous local-sensing-only swarm self-assembly, and there is no formal
convergence guarantee for every obstacle layout. The present arena has walls
and robot obstacles only. Every peer gets its own latest-sample topic to avoid
multiwriter queues retaining stale poses. Losing fresh peers or physics status
stops motion. Docking consent likewise uses one latest-sample topic per robot; empty snapshots
revoke consent. Completed edges are removed from requests. Each robot also has
an independent wall-clock velocity watchdog.

The high-level task publisher coordinates the stop/release barrier using local
state acknowledgments and actual guarded velocity commands. It does not choose
robot roles or motion paths. Once it broadcasts the new shape, the existing
peer auction and onboard controllers perform the formation.

## Simulation connector and physical meaning

The shape launch enables `formation_docking:=true` in the shared robot Xacro.
Each end has a narrow primitive stem and a spherical capture tip 110 mm from
the chassis center. The 100 mm chassis, wheels, IMU and range sensor remain.
These are **simulation-only angular-capture connectors**, not a redesign or
claim that the real robot's fixed plates can achieve these connections.

The stem and spherical tip visualize an idealized compliant magnetic connector;
they do not have rigid collision surfaces. Chassis and wheel collisions remain
enabled. Detailed connector contact/compliance is outside this abstraction. Both robots must
consent; tips must be within 18 mm, relative heading at most about 100 degrees,
linear speed below 0.025 m/s and angular speed below 0.15 rad/s. The latch then
locks the captured relative pose using a Gazebo fixed joint. Angular freedom
is available during capture, then locked; these are not freely hinged chains.
There is no modeled latch motor, magnetic force curve, breakaway load or power.
The plugin lives in `swarm_robot_dock_physics` and mutates joints only on the
Gazebo physics update thread. `/swarm/detach_all` acknowledges actual release;
use shape commands for reconfiguration or `/swarm/reset` to restore the arena,
so controllers also stop and task state is coordinated.

Each robot spans 220 mm between its two connector tips. LINE centers are
220 mm apart. CIRCLE is a ten-sided closed polygon approximating a circle.
Ten equal-length modules cannot tile four equal straight sides; SQUARE has
equal width and height with two opposite corners chamfered (eight straight
side modules plus two diagonal modules). These layouts align actual connector
positions rather than leaving half-meter gaps between robots.

Single-robot and older five-robot demos keep their original passive plates
unless `formation_docking` is explicitly enabled. No STL meshes or CAD changed.

The use of runtime cross-model joints follows Gazebo Classic's physics API;
[PAL Robotics' link-attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher)
is an established example of the same general simulation technique. This
package implements its own ROS 2 consent and capture checks.

## Interfaces and electronics abstraction

| Interface | Purpose |
| --- | --- |
| `/swarm/shape_command` (`String`) | LINE, CIRCLE, SQUARE, STOP |
| `/swarm/shape_goal` (`String` JSON) | Retained versioned task, epoch, anonymous roles |
| `/swarm/shape_request_status` (`String` JSON) | Accepted/rejected request and explanation |
| `/swarm/formation_status` (`String` JSON) | Observer: graph, progress, errors, movers |
| `/robot_N/shape/dock_request` (`String` JSON) | Latest bilateral consent snapshot: epoch, digest, robot, connections, stamp |
| `/swarm/dock_status` (`String` JSON) | Physical joint inventory, capture gaps, freshness |
| `/swarm/reset` (`Trigger`) | Stop, release, restore spawn poses, clear task |
| `/robot_N/shape/peer_state` (`String` JSON) | Bids, own pose, phase, intent and readiness |
| `/robot_N/shape/cmd_vel` (`Twist`) | Agent command consumed by its watchdog |
| `/robot_N/cmd_vel` (`Twist`) | Sole watchdog output to the differential drive |
| `/robot_N/joint_states` | Wheel angle and angular velocity, ideal encoder equivalent |
| `/robot_N/odom` | Ground-truth world odometry supplied by Gazebo |
| `/robot_N/imu/data` | IMU orientation, angular velocity, linear acceleration |
| `/robot_N/range/front` (`LaserScan`) | Lightweight three-ray front range sensor |

Do not fight the formation watchdog by publishing directly to `cmd_vel`.
The original single-robot launch remains suitable for manual drive testing.

```bash
ros2 topic echo /robot_1/joint_states --once
ros2 topic echo /robot_1/odom --once
ros2 topic echo /robot_1/imu/data --once
ros2 topic echo /robot_1/range/front --once
```

TF remains `world -> robot_N/odom -> robot_N/base_footprint -> base_link`,
with wheel, caster, IMU, front range, front dock and rear dock descendants.
The front/rear docking frames are at the capture tip centers. Physical bonds
are reported separately, avoiding cyclic TF trees when a closed ring latches.
RSP owns moving wheel transforms; exact fixed URDF transforms are refreshed
periodically for late RViz/subscriber recovery.

Gazebo's differential-drive plugin replaces the future ESP32-S3 + TB6612 +
encoder motor loop. Wheel joint positions/velocities are ideal encoders, with
no tick quantization or missed counts. Each agent represents software intended
for a robot's onboard controller. ROS 2 DDS represents Wi-Fi communication;
radio propagation, bandwidth and packet loss are not modeled. The central
computer supplies a shape; it is not a motion scheduler. The physics latch
plugin represents hardware/constraints, not a central auctioneer. Agent poses
currently use Gazebo ground truth, not camera/range-based relative localization.

## Validation

```bash
colcon test --packages-select swarm_robot_description swarm_robot_behaviors swarm_robot_docking swarm_robot_shape_formation
colcon test-result --verbose
```

An opt-in live test moves and resets the fleet, checks physical bond counts,
observes multiple simultaneous movers, verifies TF and validates latch release.
`--reconfigure` checks that subsequent shape requests preserve current positions
and release the old bonds without resetting between shapes:

```bash
python3 src/swarm_robot_shape_formation/test/verify_live.py \
  --shapes CIRCLE SQUARE LINE CIRCLE --reconfigure --report-path /tmp/connected-shapes.json
```

Use a dedicated `ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI` for automated testing.
Optional `--check-watchdog` stops/resumes an agent process briefly;
`--check-autonomy` terminates the shape publisher during assembly and restarts
it afterward. These fault tests require an explicitly isolated ROS domain.

For a separate physical-constraint test in that dedicated simulation:

```bash
python3 src/swarm_robot_shape_formation/test/verify_latch_physics.py
```

This resets the fleet and spawns two fixture copies in a clear area,
rejects distant, unilateral and mismatched-assignment capture, latches with
bilateral consent, then
perturbs one robot by 25 mm. It checks that the joint restores their relative
pose before releasing the latch and resetting again. It does not judge docking
merely from a published status label.

## Script and asset map

| File | Responsibility |
| --- | --- |
| `launch/ten_robot_shape_demo.launch.py` | Start the arena/plugin, ten namespaced robots, agents, guards, task node, observer and reset service. |
| `swarm_robot_shape_formation/shape_goal.py` | Publish anonymous templates; coordinate stopping and physical release when requests change. |
| `swarm_robot_shape_formation/shape_agent.py` | Per-robot role negotiation, peer readiness, path tracking, assembly and docking requests. |
| `swarm_robot_shape_formation/auction.py` | Single-role bid/max-consensus, deterministic ties and assignment digest. |
| `swarm_robot_shape_formation/shapes.py` | Port-connected LINE, CIRCLE and chamfered SQUARE geometry. |
| `swarm_robot_shape_formation/planning.py` | Bounded eight-neighbor A* and segment-clearance checks around peer obstacles. |
| `swarm_robot_shape_formation/concurrent.py` | Predict relative motion and sample slower/braking commands for conflicts. |
| `swarm_robot_shape_formation/peer_data.py` | Bound peer/physics freshness across clock/message delivery-order races. |
| `swarm_robot_shape_formation/consent.py` | Encode full latest consent snapshots for both ports and revoke stale intent. |
| `swarm_robot_shape_formation/velocity_guard.py` | Independently stop drive output when raw agent commands expire in wall time. |
| `swarm_robot_shape_formation/formation_monitor.py` | Read-only progress, exact physical graph completion checks and RViz markers. |
| `swarm_robot_shape_formation/static_frames.py` | Refresh fixed URDF and world/odom transforms for late inspection subscribers. |
| `config/formation.yaml` | Robot IDs, fallback spawn layout, spacing, freshness and clearance parameters. |
| `config/fastdds_udp.xml` | UDP DDS transport profile used by the launch and CLI examples. |
| `worlds/formation_arena.world` | Primitive walled arena, state-reset interface and docking world plugin. |
| `rviz/formation.rviz` | Common-world view of models, frames, odometry and formation markers. |
| `test/test_*.py` | Automated allocation, geometry, planning, freshness, consent, watchdog and transition regressions. |
| `test/verify_live.py`, `test/verify_latch_physics.py` | Opt-in live formation/reconfiguration and actual physical-latch checks. |
| `setup.py`, `setup.cfg`, `package.xml`, `resource/` | Python/ROS package installation and declared dependencies. |

`static_frames.py` is a helper used by the robot agent, not a separate installed
console executable. The physics plugin itself is supplied by
`swarm_robot_dock_physics`. Pose/control helpers and reset/spawn layout utilities
are reused from `swarm_robot_behaviors` and `swarm_robot_docking`.

## Original papers and algorithmic references

The following references identify foundations and related design ideas. The
code is a custom integration; a related paper does not automatically confer its
proofs, assumptions or hardware capabilities on this simulation.

1. Han-Lim Choi, Luc Brunet and Jonathan P. How (2009), **Consensus-Based
   Decentralized Auctions for Robust Task Allocation**, IEEE Transactions on
   Robotics, 25(4), pp. 912–926.
   [MIT publication and paper](https://dspace.mit.edu/entities/publication/b0bf0a05-be3b-433b-9f4b-ce314ed5178b),
   [DOI: 10.1109/TRO.2009.2022423](https://doi.org/10.1109/TRO.2009.2022423).
   **Algorithmic inspiration:** `auction.py` uses single-role bids and winner
   consensus, in the spirit of CBAA. It is not full multi-task CBBA. Fixed scores,
   known peers, shared epochs and all-to-all DDS simplify this implementation;
   the paper's general convergence/robustness results are not claimed here.
2. Peter E. Hart, Nils J. Nilsson and Bertram Raphael (1968), **A Formal Basis
   for the Heuristic Determination of Minimum Cost Paths**, IEEE Transactions
   on Systems Science and Cybernetics, 4(2), pp. 100–107.
   [Author-hosted paper](https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf),
   [DOI: 10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136).
   **Implemented search technique:** `planning.py` uses A* on a bounded planar
   grid. Finite search limits, changing peer obstacles and local motion control
   mean this is not a proof of globally optimal or collision-free fleet motion.
3. Mark Yim et al. (2007), **Modular Self-Reconfigurable Robot Systems
   [Grand Challenges of Robotics]**, IEEE Robotics & Automation Magazine,
   14(1), pp. 43–52.
   [Author-hosted paper](https://cs.rice.edu/~mmoll/publications/yim2007modular-self-reconfigurable-robot-systems.pdf),
   [DOI: 10.1109/MRA.2007.339623](https://doi.org/10.1109/MRA.2007.339623).
   **Conceptual background:** connected modules and changing assembly topology.
   This overview does not specify our capture ports or stop/release protocol;
   the demo remains planar and does not implement general 3D reconfiguration.

The predictor in `concurrent.py` is custom relative-motion velocity sampling;
it is not an implementation of ORCA or the full Dynamic Window Approach.
For platform citations, see the shared
[robot description references](../swarm_robot_description/README.md#original-papers-and-platform-references).

License declaration: Apache-2.0 in `package.xml`; see the workspace guide for
the current repository-wide license metadata status.
