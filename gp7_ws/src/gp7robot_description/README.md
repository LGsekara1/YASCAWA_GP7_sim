# GP7 Gazebo Pick-and-Place Workspace

This ROS 2 package contains the SolidWorks GP7 description, URDF-derived D-H
kinematics, a damped-least-squares IK solver, synchronized trapezoidal joint
trajectories, and Gazebo worlds for the EN3563 pick-and-place task.

## Build and run

```bash
cd gp7_ws
source /opt/ros/<your_ros_2_distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch gp7robot_description gazebo_sim.launch.py
```

To inspect the labelled robot and link frames in RViz from another terminal:

```bash
source /opt/ros/<your_ros_2_distro>/setup.bash
source gp7_ws/install/setup.bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix gp7robot_description)/share/gp7robot_description/rviz/gp7_frames.rviz"
```

The default world keeps Box A inside the GP7 workspace so that the complete
pick-and-place can be demonstrated. To open the task sheet's exact layout,
pass `world:=<install-prefix>/share/gp7robot_description/worlds/gp7_task_spec.sdf`.
The specified Box A destination is outside the manufacturer's 0.927 m
horizontal reach; see `docs/assignment_report.md` for the calculation and the
reachable demo alternative.

The launch file removes the crashing `gz_ros2_control` extension from the
runtime robot description and uses Gazebo Sim's native trajectory and
detachable-joint systems through `ros_gz_bridge`. The original source URDF
keeps the ROS 2 control configuration for environments where that plugin is
working. The launch leaves the arm still; it does not start the pick-and-place
planner automatically.

## Move one joint by hand

Keep the Gazebo launch running. From a second terminal, source ROS and the
workspace, then send one target angle in radians:

```bash
cd gp7_ws
source /opt/ros/<your_ros_2_distro>/setup.bash
source install/setup.bash
ros2 run gp7robot_description gp7_move_joint joint1 0.2
```

The helper reads the six current joint angles, changes only `joint1`, and sends
a 2-second trajectory that holds the other five angles. The default maximum
step is 0.5 rad (about 29 degrees). Try `Joint2`, `Joint3`, `Join4`, `Join5`,
or `Joint6` next. Use `--duration 4` for slower movement. Angles are in
radians; the helper checks the URDF limits before moving.

After you understand single-joint commands, run the assignment sequence
manually with `ros2 run gp7robot_description gp7_pick_place` while Gazebo is
open. The trajectory generator and kinematics code remain available for study.

## Assignment material

- `docs/assignment_report.md`: frame assignment, D-H values, FK/IK, velocity
  profile, reach analysis, and implementation notes.
- `gp7robot_description/kinematics.py`: FK, Jacobian, and numerical IK.
- `gp7robot_description/trajectory.py`: synchronized trapezoidal profiles.
- `gp7robot_description/pick_place.py`: upright pick-and-place sequence.
- `worlds/gp7_pick_place_reachable.sdf`: repeatable reachable demo.
- `worlds/gp7_task_spec.sdf`: Box A at the exact coordinates in the task.
