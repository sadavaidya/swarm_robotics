# Peer-relative pairing with five robots

[Workspace guide](../../README.md)

ROS 2 Humble and Gazebo Classic 11. Publish a desired pair count without choosing
robot IDs. The coordinator discovers healthy robots, uses robot-computed travel
bids to select disjoint partners, and calculates meeting poses from their current
positions. Each pair faces its front docking frames toward one another near its
own midpoint. There is no fixed arena docking location.

One pair uses two robots; two pairs use four distinct robots. Unassigned robots
stay stopped. Each pair stops when its docking geometry is satisfied. When the
requested number of pairs finishes, all five robots are stopped. Completed
pairs stay reserved until reset, and cannot be selected for another pair.

Docking in this package is **logical**: geometry checks mark a pair docked,
but no physical attachment joint is created. For ten robots forming physically
connected shapes, use [swarm_robot_shape_formation](../swarm_robot_shape_formation/README.md)
and its [latch plugin](../swarm_robot_dock_physics/README.md).

## Build and launch

```bash
cd ~/robotics/projects/Swarm_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
# Stop your previous simulation first.
env -u GTK_PATH -u GIO_MODULE_DIR \
  ros2 launch swarm_robot_docking five_robot_docking_demo.launch.py
```

Add `gui:=false rviz:=false` for headless operation. The GTK/GIO environment
workaround addresses this machine's VS Code Snap libraries. Five robots start
AVAILABLE and stopped at random starting poses, waiting for a request. The
original single-robot and leader-following launches remain available.

## Random starting poses

Random placement is enabled by default. Every launch samples fresh x/y positions
and headings for all five robot IDs across the arena. Starting robot centers are
at least **0.8 m apart**, and at least **0.5 m from each inner wall**. These are
configurable under `spawn` in `config/docking.yaml`. The current inner wall faces
are at x/y +/-5 m, so the default sampling region is [-4.5, 4.5] on each axis.

The allocator still minimizes travel and turning cost. Changing the poses changes
those costs and the selected partners; no robot ID has priority. Random placement
does not guarantee equal participation in each run.

Use a seed to reproduce an experiment:

```bash
ros2 launch swarm_robot_docking five_robot_docking_demo.launch.py spawn_seed:=0
```

The default `spawn_seed:=-1` selects a fresh seed on each launch. The actual seed
and all generated starting poses are printed in the launch log. Reusing that
seed reproduces the same requested layout. Gazebo may settle bodies by a few
millimeters after spawning.

To use the original configured fixed poses:

```bash
ros2 launch swarm_robot_docking five_robot_docking_demo.launch.py random_spawn:=false
```

Reset restores the **sampled starting layout for the current launch**, including
headings. Relaunch with the default seed to get a new layout. Sampling uses a
bounded number of attempts and raises an error if it cannot find safe placements;
it never falls back to overlapping robots. A conservative 0.12 m robot radius
also enforces minimum configurable spacing of 0.24 m and wall clearance of 0.12 m.
Bounds describe inner wall faces; update them if changing the arena world.

In every command terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/robotics/projects/Swarm_robot/install/setup.bash
```

Request **one total pair**:

```bash
ros2 topic pub --once --keep-alive 2 --qos-durability volatile /swarm/pair_count std_msgs/msg/Int32 '{data: 1}'
```

Request **two total pairs**:

```bash
ros2 topic pub --once --keep-alive 2 --qos-durability volatile /swarm/pair_count std_msgs/msg/Int32 '{data: 2}'
```

From a fresh environment, the second command selects two pairs together.
After one pair has finished, it preserves that pair and forms one additional
pair from the remaining free robots. Increasing the count while robots are
moving queues the additional pair until the existing pair finishes.
Repeating the same count is idempotent. Positive counts above 2 are capped at 2:
requesting 3 still forms two pairs and reports "3 pairs not possible".
Zero and negative counts are rejected.
Reducing the count requires reset.

The old command now requests **one additional pair** each time:

```bash
ros2 topic pub --once --keep-alive 2 --qos-durability volatile /swarm/shape_command std_msgs/msg/String "{data: 'PAIR'}"
```

Publish it twice to request two pairs. A third request keeps the goal at two
and reports that three pairs are impossible with five robots. `PAIR_2` on the same topic means two total pairs.

The explicit volatile QoS matches the allocator's command subscription.
`--keep-alive 2` keeps Humble's one-shot publisher alive for two seconds so
DDS discovery/delivery can finish. Its default keep-alive of 0.1 seconds can
lose a one-shot command under load even after printing "publishing". Check
`docking_status` to confirm acceptance. `input_pair_count` is the submitted count;
`requested_pairs` is the accepted total (at most two), and `notice` explains caps.

## Reset the environment

```bash
ros2 service call /swarm/reset std_srvs/srv/Trigger '{}'
```

Reset stops and acknowledges all controllers, clears pairs and task state,
restores every robot's launch starting position and orientation with zero
model velocity, clears agent pose caches, and enables all heartbeats. Robots
become AVAILABLE again. Reset also works during motion. A deleted Gazebo robot
is respawned using its retained robot_description. The service returns success
only after fresh odometry confirms the starting poses and all controllers are
AVAILABLE. If any step fails, motion remains held; correct the reported issue
and retry reset. Resume and unpause Gazebo before calling this service.

Reset preserves the simulation clock and wheel joint angle history. It restores
the experiment's poses, velocities, participation and pairing state; it does
not reset unrelated processes or sensor noise random seeds. Use this service
instead of Gazebo's GUI world reset for a controlled reset. If Gazebo's clock
moves backward, agents/controllers now clear old TF and pose caches, and the
allocator clears old assignments and recollects bids for the retained pair goal.
It no longer enters a permanent motion hold merely because the clock rewound.

## Inspect progress

```bash
ros2 topic echo /swarm/docking_status std_msgs/msg/String --qos-durability transient_local
ros2 topic echo /swarm/assignments --once --qos-durability transient_local
ros2 topic echo /swarm/registry --once --qos-durability transient_local
```

`docking_status` reports requested_pairs, completed_pairs, complete, status,
and a pairs list. Each pair contains its members, midpoint/orientation, targets,
status and measured separation/lateral/yaw errors. Overall DOCKED means all
requested pairs finished. Assignments carry independent per-pair generations
and final-approach/docked flags, so finishing one pair does not stop another.
RViz shows all models, TF, odometry and colored pair/owner labels.

## How it works

1. Agents publish health, world pose, front docking frame and bids at 4 Hz.
   Healthy and available are distinct: assigned/docked robots remain healthy,
   but are reserved and unavailable for new pairs.
2. The coordinator proposes partner combinations among unreserved robots.
   Each meeting origin is the midpoint between the two robot poses; its heading
   points from one proposed partner to the other. The robot-independent PAIR
   template is rotated and translated to this origin.
3. Agents bid distance-to-target plus weighted heading error. After a 0.6 s
   collection window, the coordinator chooses the minimum total bid among
   disjoint partner combinations. Meeting centers must be at least 0.7 m apart
   and away from existing pair centers. This is exact for this five-robot demo.
4. Each controller approaches its staging pose, aligns, waits for its partner,
   and makes a slow final approach. Staging distance is reduced for close peers.
5. Success uses actual front docking frame poses: separation below 40 mm,
   lateral offset below 20 mm, opposing headings within 5 degrees, base poses
   near their targets, and a 0.5 simulation-second dwell. Both controllers stop.

The primitive front plate and prefixed mating frames are reused unchanged.
The front face is 62 mm ahead of base_link, with a nominal 12 mm mating gap.
The controller accounts for the inherited drive axle's 20 mm forward offset.
Motion is capped at 0.15 m/s normally, 0.035 m/s finally, and 0.6 rad/s turning.

The coordinator is centralized; bid computation and motion control run in the
individual robot namespaces. Partner discovery uses ROS DDS and simulated
world odometry, not random roaming or visual identification. DDS represents
Wi-Fi messaging; there is no radio propagation simulation. World pose is
privileged Gazebo ground truth, not realistic physical localization.

This is logical docking: no weld, latch or magnet attaches bodies. The robots
face one another and stop with a small gap. The experiment assumes a clear
arena. Meeting-center clearance is not collision avoidance or path planning;
arbitrary crowded placements and an obstructing failed robot need a future
planner. The inherited narrow elevated range scanner is not used for docking.
A hard-killed controller can leave the drive plugin's last command active; normal
shutdown sends zero and stale data/health/assignments stop an active controller.

## Interfaces

| Interface | Type | Meaning |
| --- | --- | --- |
| /swarm/pair_count | std_msgs/Int32 | Desired total pairs; positive requests above 2 cap at 2 |
| /swarm/shape_command | std_msgs/String | PAIR adds one; PAIR_2 requests two total |
| /swarm/shape | String JSON v1 | Bid epoch, candidate rendezvous and slots |
| /swarm/robot_state | String JSON v1 | Agent health, availability, pose, bids and phase |
| /swarm/registry | String JSON v1 | Healthy/occupied/available robots |
| /swarm/assignments | String JSON v1 | Leased per-robot, per-pair targets and permissions |
| /swarm/docking_status | String JSON v1 | Pair progress and completion |
| /swarm/markers | visualization_msgs/MarkerArray | Pair and owner labels |
| /swarm/reset | std_srvs/Trigger | Restore the experiment |
| /robot_N/set_available | std_srvs/SetBool | Disable/enable participation |
| /robot_N/swarm/reset | std_srvs/Trigger | Internal agent cache/participation reset |
| /swarm/hold | std_srvs/SetBool | Internal reset motion hold and task clearing |
| /swarm/simulation/set_entity_state | gazebo_msgs/SetEntityState | Gazebo world plugin used by reset |

Each namespace retains cmd_vel, odom, joint_states, imu/data, range/front,
robot_description and the existing prefixed TF tree including front_dock_link
and front_dock_frame. Wheel joint positions/velocities simulate encoders; the
Gazebo differential-drive plugin represents the future ESP32/TB6612 drive layer.

To test failure, inspect members then disable an assigned robot early:

```bash
ros2 service call /robot_2/set_available std_srvs/srv/SetBool '{data: false}'
```

Local motion stops and heartbeats cease. Brief pose/TF gaps reserve an assigned
robot for up to the 1.8-second health timeout; cached poses cannot authorize
movement or docking. This prevents false replacement of a completed pair.
After 1.8 simulation seconds the
coordinator reopens that slot, keeps the surviving partner's target, and bids
among remaining unreserved robots for a replacement. It reports insufficient
health if the requested pair count cannot be supported. Rejoining robots can
fill pending tasks. A disabled model remains an obstacle; reset re-enables it.

## Files and validation

- `spawn_layout.py`: bounded random sampling, wall clearance and robot separation.
- `rendezvous.py`: peer-relative proposals and disjoint minimum-bid selection.
- `swarm_allocator.py`: count requests, reserved pairs, repair and completion.
- `swarm_agent.py`, `docking_controller.py`: namespaced bids and local motion.
- `environment_reset.py`: coordinated stop, pose restoration and respawn.
- `worlds/docking_arena.world`: reused primitive arena plus Gazebo state plugin.
- `launch/five_robot_docking_demo.launch.py`: reset node and five reused robots.
- `config/docking.yaml`: starting poses, clearance, timings and gains.
- `test/test_rendezvous.py`: geometry/disjoint-selection regressions.
- `test/test_spawn_layout.py`: spacing, reproducibility, impossible layouts and selection variation.
- `test/verify_live.py`: live one/additional/two-pair, repeat and reset scenarios.
- `package.xml`, `setup.py`: dependencies, executable and world installation.

```bash
colcon test
colcon test-result --verbose
# Dedicated docking simulation only; this test moves and resets its robots:
python3 src/swarm_robot_docking/test/verify_live.py
python3 src/swarm_robot_docking/test/verify_live.py --failure-only
python3 src/swarm_robot_docking/test/verify_command_recovery.py
```

Validated on 2026-10-07: all three packages built, 34 tests passed with no
failures, and docking Python lint passed. Headless Gazebo checks passed for one
pair, an additional pair while preserving the first, two pairs from one request,
repeated-count idempotence, reset after docking/during motion, disabled-agent
recovery, deleted-model respawn, failed-partner replacement, and insufficient
health followed by rejoin. With `random_spawn:=false` the selected pairs
were robot_2/robot_4 and robot_3/robot_5, meeting near (-1.4, 0.4) and (1.4, 0.4).
Measured final face gaps were about 22–25 mm, with all five cmd_vel outputs zero.

The random-placement milestone added ten tests and reported 44 passing tests. The random-layout module, launch and test files pass lint.
Random placement validation is saved in `log/random_spawn_validation/`.
Pairing evidence is saved in workspace
`log/relative_pair_validation/`. Previous fixed-location docking and unchanged
leader-following validation evidence is retained under `log/docking_validation/`.
To isolate another simulation, export a unique ROS_DOMAIN_ID and
GAZEBO_MASTER_URI in both launch and command terminals.

Pair-command recovery validation is saved in `log/pair_command_fix_validation/`.
That pairing milestone reported **56 tests, zero errors/failures** across the then-existing workspace. Live checks
verify an oversized request completes two pairs and stops all five robots,
Gazebo clock rewind recovers automatically, and a count of two works after reset.
The dropped default CLI publication and transient pose-gap failures were
reproduced before applying the command flags and assignment reservation fix.

## Reused components and script roles

The file list above covers the current pairing path. `protocol.py` contains
versioned JSON envelopes, and `pose_provider.py` handles fresh ground-truth
odometry/TF with a bounded arrival cache. `shape_library.py` supplies shared
pose/bid helpers; `auction.py` retains the small exact slot-allocation utility.
The peer-relative multi-pair demo clears disjoint pair offers in `rendezvous.py`.
Unit tests are in `test/test_*.py`; `test/verify_*.py` scripts are opt-in live
checks that may move/reset their dedicated test fleet.

The ten-robot formation package reuses random spawn generation and the reset
node. Reset can optionally release physical joints first, but enabling that
reuse does not turn this package's five-robot demo into a physical docking demo.

## Original papers and algorithmic background

These references ground the bidding/allocation design. The pair selector is a
custom exact search over small disjoint pair offers; it does not reproduce the
papers' full protocols or their experimental systems.

1. Brian P. Gerkey and Maja J. Matarić (2002), **Sold!: Auction Methods for
   Multirobot Coordination**, IEEE Transactions on Robotics and Automation,
   18(5), pp. 758–768.
   [Original publication](https://robotics.usc.edu/publications/10/),
   [DOI: 10.1109/TRA.2002.803462](https://doi.org/10.1109/TRA.2002.803462).
   **Conceptual inspiration:** robot bids and publish/subscribe task negotiation.
   Our coordinator clears pair bids centrally; it is not the paper's distributed
   MURDOCH system or a decentralized consensus auction.
2. Brian P. Gerkey and Maja J. Matarić (2004), **A Formal Analysis and Taxonomy
   of Task Allocation in Multi-Robot Systems**, The International Journal of
   Robotics Research, 23(9), pp. 939–954.
   [Author-hosted paper](https://robotics.stanford.edu/~gerkey/research/final_papers/mrta-taxonomy.pdf),
   [DOI: 10.1177/0278364904045564](https://doi.org/10.1177/0278364904045564).
   **Problem background:** separating task allocation from execution. Pairing
   requires two robots per task; our small matching problem is custom and does
   not claim general optimality for the full docking/control problem.

License declaration: Apache-2.0 in `package.xml`; see the workspace guide for
the current repository-wide license metadata status.
