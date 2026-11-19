# YASKAWA GP7 Pick-and-Place Simulation (ROS 2 + Gazebo Sim)

A ROS 2 / Gazebo Sim simulation of the **YASKAWA MOTOMAN-GP7** 6-axis arm picking a 5 cm cube (**Box B**) and placing it upright on the centre of a 30 × 30 × 20 cm support (**Box A**). Built for the EN3563 mini project.

The package contains the SolidWorks-exported GP7 model, D-H forward/inverse kinematics (damped least-squares IK), synchronized S-curve trajectories, a live monitoring/control dashboard, and two Gazebo worlds.

---

## Table of contents

1. [Gallery](#gallery)
2. [Repository layout](#repository-layout)
3. [Requirements](#requirements)
4. [Build and run](#build-and-run)
5. [System architecture](#system-architecture)
6. [ROS 2 nodes](#ros-2-nodes)
7. [Topics, publishers and subscriptions](#topics-publishers-and-subscriptions)
8. [Message data reference](#message-data-reference)
9. [Parameters and launch arguments](#parameters-and-launch-arguments)
10. [Robot model data](#robot-model-data)
11. [Simulation world data](#simulation-world-data)
12. [Pick-and-place sequence](#pick-and-place-sequence)
13. [Dashboard (`gp7_gui`)](#dashboard-gp7_gui)
14. [Move one joint by hand](#move-one-joint-by-hand)
15. [Documentation](#documentation)
16. [License](#license)

---

## Gallery

> Add your screenshots to `docs/images/` using the file names below (or change the paths). Each block has a ready-made slot.

### 1. Coordinate frames in RViz

Launch: `ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix gp7robot_description)/share/gp7robot_description/rviz/gp7_frames.rviz"`

<!-- Replace the file with your own screenshot -->
![GP7 link frames in RViz](docs/images/rviz_frames.png)

*Suggested capture: RobotModel + TF display with axes and frame names (`base_link`, `Link 1` … `Link6`), Fixed Frame = `base_link`.*


### 2. Dashboard

Launch: `ros2 run gp7robot_description gp7_gui`

<!-- Replace the file with your own screenshot -->
![GP7 pick-and-place dashboard](docs/images/dashboard.png)

*Suggested capture: window title "GP7 pick-and-place" showing joint angle/velocity plots, gripper top-view path, and height/speed plot while the task runs.*

### 3. Robot workspace in Gazebo

Launch: `ros2 launch gp7robot_description gazebo_sim.launch.py`

<!-- Replace the file with your own screenshot -->
![GP7 workspace in Gazebo](docs/images/gazebo_workspace.png)

*Suggested capture: GP7 at the origin, blue Box A at (0.5, 0.5), orange Box B at (0.35, 0.25).*

| Pick (Box B grasped) | Carry | Place (Box B on Box A) |
|---|---|---|
| ![Pick](docs/images/gazebo_pick.png) | ![Carry](docs/images/gazebo_carry.png) | ![Place](docs/images/gazebo_place.png) |

### 4. Reach / workspace plot (optional)

![Reachability](docs/images/reachability.png)

The repository already ships a reachability figure at `gp7_ws/src/gp7robot_description/docs/learning_guide/figures/reachability.png`; copy it to `docs/images/` or point the link there.

---

## Repository layout

```
yascawa_gp7_sim/
├── LICENSE
├── Task_sheet.pdf                      # original assignment sheet
├── GP7 (Solidworks 2020).zip           # CAD source
├── Resources-20261116.zip              # supplied resources
├── gp7_robot_description/              # raw SolidWorks URDF export (ROS 1 style, reference only)
│   ├── urdf/ meshes/ config/ launch/ export.log
└── gp7_ws/                             # ROS 2 workspace
    └── src/gp7robot_description/       # ament_python package
        ├── urdf/GP7.urdf               # robot description (with ros2_control block)
        ├── meshes/                     # base_link, Link 1, Link2 … Link6 (.STL)
        ├── rviz/gp7_frames.rviz        # RViz config: model + TF frames
        ├── worlds/
        │   ├── gp7_pick_place_reachable.sdf
        │   └── gp7_task_spec.sdf
        ├── launch/
        │   ├── gazebo_sim.launch.py    # Gazebo + bridge + spawn + demo
        │   └── display.launch.py       # RViz + joint_state_publisher_gui
        ├── gp7robot_description/
        │   ├── kinematics.py           # D-H table, FK, Jacobian, DLS IK, limits
        │   ├── trajectory.py           # S-curve profiles, blending, JointTrajectory builder
        │   ├── pick_place.py           # pick-and-place controller node
        │   ├── manual_joint.py         # single-joint command node
        │   ├── reset_world.py          # reset Box B + home arm
        │   ├── workspace.py            # offline reachability check / random Box B
        │   └── gui.py                  # Tk + matplotlib dashboard node
        └── docs/                       # assignment_report.md, learning guide (PDF/HTML + figures)
```

## Requirements

- ROS 2 with Gazebo Sim (`gz sim`) and the `ros_gz` packages (`ros_gz_sim`, `ros_gz_bridge`)
- `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`
- Python: `numpy`, `matplotlib`, `python3-tk` (dashboard)
- Message packages: `sensor_msgs`, `trajectory_msgs`, `std_msgs`, `rosgraph_msgs`

Install dependencies with `rosdep install --from-paths src --ignore-src -r -y`.

## Build and run

```bash
cd gp7_ws
source /opt/ros/<your_ros_2_distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Gazebo + bridges + robot + automatic pick-and-place (run_demo defaults to true)
ros2 launch gp7robot_description gazebo_sim.launch.py
```

Common variants:

```bash
# Start the scene only; drive it from the dashboard or by hand
ros2 launch gp7robot_description gazebo_sim.launch.py run_demo:=false

# Server only, no Gazebo GUI
ros2 launch gp7robot_description gazebo_sim.launch.py headless:=true

# Task-sheet world
ros2 launch gp7robot_description gazebo_sim.launch.py \
  world:=<install-prefix>/share/gp7robot_description/worlds/gp7_task_spec.sdf

# RViz-only model inspection with sliders
ros2 launch gp7robot_description display.launch.py

# Labelled link frames in RViz (while Gazebo is running)
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix gp7robot_description)/share/gp7robot_description/rviz/gp7_frames.rviz"
```

Executables installed by the package:

| Command | Entry point |
|---|---|
| `ros2 run gp7robot_description gp7_pick_place` | `pick_place:main` |
| `ros2 run gp7robot_description gp7_move_joint <joint> <rad>` | `manual_joint:main` |
| `ros2 run gp7robot_description gp7_reset_world` | `reset_world:main` |
| `ros2 run gp7robot_description gp7_gui` | `gui:main` |

> The launch file strips the `gz_ros2_control` plugin from the runtime URDF and instead uses Gazebo Sim's native `JointTrajectoryController`, `JointStatePublisher` and `DetachableJoint` systems, bridged with `ros_gz_bridge`. The source `GP7.urdf` keeps its `ros2_control` block for setups where that plugin works.

## System architecture

```
                          ┌────────────────────────── Gazebo Sim (gz sim) ──────────────────────────┐
                          │  JointTrajectoryController   JointStatePublisher   DetachableJoint      │
                          └───────▲──────────────────────────────┬────────────────────▲─────────────┘
                                  │ gz.msgs.JointTrajectory      │ gz.msgs.Model      │ gz.msgs.Empty
                      ┌───────────┴──────────────────────────────▼────────────────────┴─────────────┐
                      │                       ros_gz_bridge  (parameter_bridge)                      │
                      └───▲────────────────────┬──────────────────▲──────────────────▲──────────────┘
      /model/GP7Robot/    │          /joint_states, /clock         │ /gp7/gripper/    │ /gp7/gripper/
      joint_trajectory    │                    │                   │ attach           │ detach
   ┌──────────────────────┴───┐   ┌───────────▼───────────┐   ┌───┴──────────────────┴───┐
   │ gp7_pick_place_controller│   │  robot_state_publisher│   │ gp7_reset_world (detach) │
   │ gp7_manual_joint_commander│  │  → /tf, /tf_static    │   │ gp7_gui (monitor + runs  │
   │ gp7_reset_world          │   │  → /robot_description │   │ the three CLI nodes)     │
   └──────────────────────────┘   └───────────────────────┘   └──────────────────────────┘
```

## ROS 2 nodes

### Launched by `gazebo_sim.launch.py`

| Node / process | Package · executable | Purpose | Started |
|---|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | Publishes TF and `/robot_description` from the simulation URDF (`use_sim_time: true`) | immediately |
| Gazebo Sim | `gz sim -r -v 2 <world>` (`-s` if `headless:=true`) | Physics, world, joint controller (not a ROS node) | immediately |
| `parameter_bridge` | `ros_gz_bridge` | Bridges Gazebo topics ⇄ ROS 2 topics (see below) | immediately |
| `create` (spawner) | `ros_gz_sim` | Spawns `GP7Robot` from `robot_description` at (0, 0, 0) | +5 s |
| `gp7_pick_place_controller` | `gp7robot_description` · `gp7_pick_place` | Plans and runs the pick-and-place (`use_sim_time: true`) | +10 s, only if `run_demo:=true` |

### Launched by `display.launch.py`

| Node | Package · executable | Purpose |
|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | TF from `GP7.urdf` |
| `joint_state_publisher_gui` | `joint_state_publisher_gui` | Slider GUI that publishes `/joint_states` |
| `rviz2` | `rviz2` | Visualisation |

### Run on demand

| Node name | Executable | Purpose |
|---|---|---|
| `gp7_pick_place_controller` | `gp7_pick_place` | S-curve / Cartesian planner; publishes trajectories and gripper signals |
| `gp7_manual_joint_commander` | `gp7_move_joint` | Moves one joint to a target angle, holds the other five |
| `gp7_reset_world` | `gp7_reset_world` | Detaches the gripper, teleports Box B, homes the arm |
| `gp7_gui` | `gp7_gui` | Monitors `/joint_states`, plots, and launches the three nodes above |

## Topics, publishers and subscriptions

### Complete topic table

| Topic | Type | Direction | Publisher(s) | Subscriber(s) |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | Gazebo `JointStatePublisher` via `parameter_bridge` (remapped from `/world/gp7_pick_place/model/GP7Robot/joint_state`). In `display.launch.py`: `joint_state_publisher_gui` | `robot_state_publisher`, `gp7_pick_place_controller`, `gp7_manual_joint_commander`, `gp7_reset_world`, `gp7_gui` (queue depth 50) |
| `/model/GP7Robot/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | ROS → Gazebo | `gp7_pick_place_controller`, `gp7_manual_joint_commander`, `gp7_reset_world` | `parameter_bridge` → Gazebo `JointTrajectoryController` |
| `/gp7/gripper/attach` | `std_msgs/msg/Empty` | ROS → Gazebo | `gp7_pick_place_controller` | `parameter_bridge` → Gazebo `DetachableJoint` (attach) |
| `/gp7/gripper/detach` | `std_msgs/msg/Empty` | ROS → Gazebo | `gp7_pick_place_controller`, `gp7_reset_world` | `parameter_bridge` → Gazebo `DetachableJoint` (detach) |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | `parameter_bridge` | all nodes using `use_sim_time` (`robot_state_publisher`, `gp7_pick_place_controller`) |
| `/robot_description` | `std_msgs/msg/String` | ROS | `robot_state_publisher` | `ros_gz_sim create` (spawn), RViz `RobotModel` display |
| `/tf` | `tf2_msgs/msg/TFMessage` | ROS | `robot_state_publisher` | RViz `TF` display |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | ROS | `robot_state_publisher` | RViz `TF` display |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz tool | RViz `SetInitialPose` tool | none in this package |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz tool | RViz `SetGoal` tool | none in this package |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | RViz tool | RViz `PublishPoint` tool | none in this package |

The last three are RViz default tools enabled in `gp7_frames.rviz`; nothing in this package consumes them.

### `ros_gz_bridge` configuration

| Bridge argument | Direction | ROS type | Gazebo type |
|---|---|---|---|
| `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` | Gazebo → ROS | `rosgraph_msgs/msg/Clock` | `gz.msgs.Clock` |
| `/model/GP7Robot/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory` | ROS → Gazebo | `trajectory_msgs/msg/JointTrajectory` | `gz.msgs.JointTrajectory` |
| `/gp7/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty` | ROS → Gazebo | `std_msgs/msg/Empty` | `gz.msgs.Empty` |
| `/gp7/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty` | ROS → Gazebo | `std_msgs/msg/Empty` | `gz.msgs.Empty` |
| `/world/gp7_pick_place/model/GP7Robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model` | Gazebo → ROS | `sensor_msgs/msg/JointState` | `gz.msgs.Model` |

Remapping: `/world/gp7_pick_place/model/GP7Robot/joint_state` → `/joint_states`.

### Per-node publications and subscriptions

| Node | Publishes | Subscribes |
|---|---|---|
| `gp7_pick_place_controller` | `/model/GP7Robot/joint_trajectory` (depth 10), `/gp7/gripper/attach` (10), `/gp7/gripper/detach` (10) | `/joint_states` (10), `/clock` |
| `gp7_manual_joint_commander` | `/model/GP7Robot/joint_trajectory` (10) | `/joint_states` (10) |
| `gp7_reset_world` | `/model/GP7Robot/joint_trajectory` (10), `/gp7/gripper/detach` (10) | `/joint_states` (10) |
| `gp7_gui` | none | `/joint_states` (50) |
| `robot_state_publisher` | `/tf`, `/tf_static`, `/robot_description` | `/joint_states`, `/clock` |
| `parameter_bridge` | `/clock`, `/joint_states` | `/model/GP7Robot/joint_trajectory`, `/gp7/gripper/attach`, `/gp7/gripper/detach` |

### Gazebo-side interfaces (not ROS topics)

| Interface | Used by | Purpose |
|---|---|---|
| `gz service /world/gp7_pick_place/set_pose` (`gz.msgs.Pose` → `gz.msgs.Boolean`, 3 s timeout) | `gp7_reset_world` (via `subprocess`) | Teleports `BoxB` to its start pose with identity orientation |
| Gazebo plugin `gz-sim-joint-trajectory-controller-system` | robot URDF in launch file | Executes joint trajectories (PID, see [Robot model data](#robot-model-data)) |
| Gazebo plugin `gz-sim-joint-state-publisher-system` | robot URDF in launch file | Publishes joint states |
| Gazebo plugin `gz-sim-detachable-joint-system` | robot URDF in launch file | Attaches/detaches `BoxB`/`link` to `Link6` on the attach/detach topics |

## Message data reference

### `/joint_states` (`sensor_msgs/JointState`)

| Field | Content |
|---|---|
| `header.stamp` | Simulation time |
| `name[]` | `joint1`, `Joint2`, `Joint3`, `Join4`, `Join5`, `Joint6` (names may arrive with `/` or `::` prefixes; nodes strip them with `name.replace("/", "::").split("::")[-1]`) |
| `position[]` | Joint angles in rad |

All consuming nodes match joints **by name**, so message order does not matter. Messages missing any of the six joint names are ignored.

> Joint names come from the CAD export and contain irregular spelling: `joint1`, `Joint2`, `Joint3`, **`Join4`**, **`Join5`**, `Joint6`. Use them exactly as shown.

### `/model/GP7Robot/joint_trajectory` (`trajectory_msgs/JointTrajectory`)

| Field | Content |
|---|---|
| `header.stamp` | Current ROS (sim) time when the message is built |
| `joint_names` | The six joint names above, in order |
| `points[i].positions` | Six joint angles (rad) |
| `points[i].velocities` | Six joint velocities (rad/s); zero at start and end of each move |
| `points[i].time_from_start` | Seconds, as `Duration(sec, nanosec)` |

Sampling: **5 ms** between points (`SAMPLE_PERIOD = 0.005`). Gazebo's controller does not interpolate between points, so dense sampling keeps the PID D-term from spiking.

Manual-joint moves send a single point: goal positions, zero velocities, at `time_from_start = --duration` (default 2.0 s).

### `/gp7/gripper/attach`, `/gp7/gripper/detach` (`std_msgs/Empty`)

No payload. Each signal is published in repeats of 0.25 s spacing: 3× for initial release, grab and release in the pick-and-place; 1× in `gp7_reset_world`.

### `/clock` (`rosgraph_msgs/Clock`)

Simulation time; all waits and trajectory timing in the controller use it.

## Parameters and launch arguments

### Launch arguments (`gazebo_sim.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `world` | `<share>/worlds/gp7_pick_place_reachable.sdf` | World SDF to load |
| `headless` | `false` | `true` runs the Gazebo server only (`gz sim -s`) |
| `run_demo` | `true` | Start `gp7_pick_place` automatically after 10 s |

Environment: the launch file prepends the package parent directory to `GZ_SIM_RESOURCE_PATH` so meshes resolve.

### `gp7_pick_place_controller` parameters

| Parameter | Default | Meaning |
|---|---|---|
| `box_a_x` | 0.50 m | Box A centre x |
| `box_a_y` | 0.50 m | Box A centre y |
| `box_a_top_z` | 0.20 m | Top surface height of Box A |
| `box_b_x` | 0.35 m | Box B start x |
| `box_b_y` | 0.25 m | Box B start y |
| `box_b_center_z` | 0.025 m | Box B centre height (half of 0.05 m cube) |
| `clearance_height` | 0.55 m | Hover height above Box B for lift and joint-1 rotation |
| `retreat_height` | 0.15 m | Vertical retreat after release |
| `speed_scale` | 0.30 | Fraction of max joint speed used for joint-space moves (0 < s ≤ 1) |
| `cartesian_speed` | 0.25 m/s | Tool speed limit on straight-line moves |
| `cartesian_acceleration` | 0.40 m/s² | Tool acceleration limit |
| `blend_radius` | 0.10 m | Corner arc radius for multi-leg paths |

Example:

```bash
ros2 run gp7robot_description gp7_pick_place --ros-args \
  -p use_sim_time:=true -p box_b_x:=0.40 -p box_b_y:=-0.20
```

### `gp7_reset_world` parameters

| Parameter | Default |
|---|---|
| `world_name` | `gp7_pick_place` |
| `box_b_x` | 0.35 m |
| `box_b_y` | 0.25 m |
| `box_b_center_z` | 0.025 m |

### `gp7_move_joint` arguments

| Argument | Default | Meaning |
|---|---|---|
| `joint` | required | One of `joint1 Joint2 Joint3 Join4 Join5 Joint6` |
| `target_rad` | required | Target angle in rad (checked against joint limits) |
| `--duration` | 2.0 s | Move duration |
| `--max-step` | 0.5 rad (~29°) | Largest allowed change per command |

## Robot model data

### Links and joints

Kinematic chain (parent → child), as in `GP7.urdf`:

```
world ─(gp7_world_fixed, sim only)→ base_link ─joint1→ "Link 1" ─Joint2→ Link2 ─Joint3→ Link3
      ─Join4→ Link4 ─Join5→ Link5 ─Joint6→ Link6 ─(gp7_gripper_mount)→ gp7_gripper
                                                   ├─ gp7_left_finger
                                                   └─ gp7_right_finger
```

Note the first link is named **`Link 1`** (with a space) in the URDF.

Frames added only in the simulation URDF (generated in `gazebo_sim.launch.py`):

| Frame | Details |
|---|---|
| `world` | Fixed parent of `base_link` (`gp7_world_fixed`), marked static |
| `gp7_gripper` | 0.12 kg, mounted on `Link6` 0.075 m along the flange axis (upright reference orientation) |
| `gp7_left_finger` | 0.05 kg, box 0.010 × 0.050 × 0.100 m, offset (-0.0325, 0, 0.025) m in gripper frame |
| `gp7_right_finger` | 0.05 kg, same box, offset (0.0375, 0, 0.025) m |

### D-H parameters (standard convention, `A_i = Rz(θ_i) Tz(d_i) Tx(a_i) Rx(α_i)`)

| i | GP7 axis / URDF joint | θ offset (°) | d (m) | a (m) | α (°) |
|---:|---|---:|---:|---:|---:|
| 1 | S / `joint1` | -112.650760 | 0.330000 | 0.040000 | -90 |
| 2 | L / `Joint2` | -104.091531 | 0.000000 | 0.445000 | 0 |
| 3 | U / `Joint3` | 174.798566 | 0.000000 | 0.040000 | -90 |
| 4 | R / `Join4` | -9.999933 | 0.440000 | 0.000000 | 90 |
| 5 | B / `Join5` | -177.345550 | 0.000000 | 0.000000 | -90 |
| 6 | T / `Joint6` | 0 | -0.058893 | 0.000000 | 0 |

A fixed rotation (`DH_TO_LINK6_ROTATION`) maps the D-H wrist frame to the CAD `Link6` frame, so `forward_kinematics()` matches the URDF/TF frame.

### Joint limits, speed and acceleration

| Joint | Lower (rad) | Upper (rad) | Max speed (°/s) | Accel. limit (rad/s²) |
|---|---:|---:|---:|---:|
| `joint1` (S) | -1.76541 | 4.16871 | 375 | 0.45 |
| `Joint2` (L) | -1.38041 | 1.84845 | 315 | 0.45 |
| `Joint3` (U) | -0.08375 | 4.45411 | 410 | 0.55 |
| `Join4` (R) | -3.31613 | 3.31613 | 550 | 0.80 |
| `Join5` (B) | -2.40252 | 2.30987 | 550 | 0.80 |
| `Joint6` (T) | -2.42329 | 3.85641 | 1000 | 1.00 |

Speeds come from the GP7 datasheet. `Joint3` limits were mirrored from the CAD export so the forearm cannot fold into the upper arm; `Join4` was widened to the datasheet ±190°.

Maximum horizontal reach used by the planner: **0.927 m**.

### Gazebo joint controller (PID, from `gazebo_sim.launch.py`)

| Joint | P | I | D | I clamp | Command limit (N·m) |
|---|---:|---:|---:|---:|---:|
| `joint1` | 180 | 0 | 28 | 1 | ±260 |
| `Joint2` | 180 | 120 | 28 | 60 | ±260 |
| `Joint3` | 150 | 100 | 24 | 45 | ±220 |
| `Join4` | 35 | 0 | 7 | 1 | ±80 |
| `Join5` | 25 | 8 | 5 | 6 | ±55 |
| `Joint6` | 15 | 0 | 3 | 1 | ±35 |

Integral action is used only on gravity-loaded joints (2, 3, 5). Initial position of all joints is 0. Self-collision is enabled.

### Reference configurations

| Name | Value (rad) |
|---|---|
| Home (reset target) | `[0, 0, 0, 0, 0, 0]` |
| `UPRIGHT_REFERENCE_CONFIGURATION` | `[2.58637615, 0.37832553, 3.91332751, 0.17453176, 0.71113281, 0.0]` (flange pointing straight down above Box B) |

## Simulation world data

World name: **`gp7_pick_place`** (used in service and bridge topic names). Physics step 0.001 s, real-time factor 1.0. Plugins: Physics, UserCommands, SceneBroadcaster. A 5 × 5 m ground plane and a directional sun light are included.

| Object | Pose (x, y, z) m | Size (m) | Notes |
|---|---|---|---|
| Robot `GP7Robot` | (0, 0, 0) | n/a | Spawned by `ros_gz_sim create` |
| `BoxA` | (0.50, 0.50, 0.10) | 0.30 × 0.30 × 0.20 | Static, blue; top surface at z = 0.20 |
| `BoxB` | (0.35, 0.25, 0.025) | 0.05 × 0.05 × 0.05 | Dynamic, orange, 0.125 kg, friction µ = 1.2 |

Place target for Box B centre: `(0.50, 0.50, 0.225)`.

Worlds:

- `worlds/gp7_pick_place_reachable.sdf`: default repeatable demo layout.
- `worlds/gp7_task_spec.sdf`: task-sheet world. In the current files both worlds contain the same geometry (Box A at 0.5, 0.5, the lecturer's revision of the task sheet's (1, 1, 0), which lies beyond the 0.927 m reach); they differ only in comments. See `docs/assignment_report.md` for the reach analysis.

## Pick-and-place sequence

Implemented in `pick_place.py` (`PickPlaceController.run_demo`):

1. **Wait for bridge**: up to 15 s for subscribers on the trajectory/detach topics and a valid `/joint_states`.
2. **Initial release**: 3× `/gp7/gripper/detach`.
3. **Lift and rotate**: joint-space S-curve to a hover pose 0.55 m above Box B, with joint 1 rotation blended in once the lift is ~2/3 done.
4. **Lower** to Box B on a straight vertical line (gripper upright, IK solved at every sample).
5. **Grab**: 3× `/gp7/gripper/attach`.
6. **Carry**: raise → horizontal move above Box A → lower onto Box A, as one blended path with rounded corners (radius 0.10 m) and a single speed profile; release pose is 5 mm above Box A's top.
7. **Release**: 3× `/gp7/gripper/detach`, wait 0.5 s.
8. **Retreat**: lift 0.15 m straight up and stop.

Safety checks: reach limit (0.927 m), IK branch-switch guard (max 5° per sample), joint-speed limit scaling, waypoint tolerance 2° (8° abort limit, 15° for joint 6), 0.25 s settle time.

## Dashboard (`gp7_gui`)

Start Gazebo with `run_demo:=false`, then run `ros2 run gp7robot_description gp7_gui`.

**Controls**

| Control | Action |
|---|---|
| Box B start x / y (m, ±0.95) | Start position for the next run (defaults 0.35, 0.25) |
| Place Box B at random | Finds a reachable random position (`workspace.random_box_b`), teleports Box B there, homes the arm |
| Start task (pick & place) | Runs `gp7_pick_place` with `box_b_x`, `box_b_y` |
| Reset world | Runs `gp7_reset_world` with the typed x/y |
| Stop | Sends SIGINT to the running command |
| Clear graphs | Clears the buffers |

**Live data (from `/joint_states` only)**

| Plot / readout | Content |
|---|---|
| Joint angles | J1–J6 in deg |
| Joint velocities | Finite difference of positions, deg/s |
| Gripper path (top view) | x/y of the gripper via forward kinematics, with the 0.927 m reach circle, Box A and Box B |
| Gripper height and speed | z (m) and tool speed (m/s) |
| Readouts | Sim time, per-joint angle and speed, gripper x/y/z |

Settings: 30 s history, at most 50 samples/s stored, 100 ms refresh. If simulation time jumps backwards (Gazebo restarted), buffers are cleared automatically.

## Move one joint by hand

With Gazebo running:

```bash
ros2 run gp7robot_description gp7_move_joint joint1 0.2
ros2 run gp7robot_description gp7_move_joint Join4 0.3 --duration 4
```

The command reads the current six angles, changes only the chosen joint and holds the rest.

## Documentation

- `gp7_ws/src/gp7robot_description/docs/assignment_report.md`: frame assignment, D-H table, FK/IK, velocity profile, reach analysis
- `gp7_ws/src/gp7robot_description/docs/GP7_learning_guide.pdf` and `docs/learning_guide/guide.html`: learning guide with figures (`architecture.png`, `ros_topics.png`, `task_timeline.png`, `frames_2d.png`, and more)
- `Task_sheet.pdf`: original assignment

## License

See [LICENSE](LICENSE) (package metadata declares Apache-2.0).
