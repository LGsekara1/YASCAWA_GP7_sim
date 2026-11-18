"""ROS 2 pick-and-place controller for the GP7 Gazebo demo.

Motion plan
-----------
1. Joint-space moves (no box held) bring the arm from home to a hover pose
   above Box B on the "upright" IK branch.
2. Every motion while the gripper is near or holding Box B is a straight
   Cartesian line with the Link6 orientation held constant. The path
   parameter follows a trapezoidal velocity profile and inverse kinematics
   is solved at every sample, so Box B stays upright for the whole motion.
3. After release the arm retreats vertically and returns home.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .kinematics import (
    JOINT_NAMES,
    MAX_JOINT_SPEEDS,
    UPRIGHT_REFERENCE_CONFIGURATION,
    forward_kinematics,
    horizontal_reach,
    inverse_kinematics,
    pose,
)
from .trajectory import TrajectorySample, trapezoidal_segment


MAX_HORIZONTAL_REACH = 0.927
BOX_B_SIZE = (0.05, 0.05, 0.05)
GRIPPER_LINK_OFFSET = 0.075
SAFE_SPEED_SCALE = 0.15
SAMPLE_PERIOD = 0.02
# Settled when every joint is within this of its goal for SETTLE_TIME seconds.
WAYPOINT_TOLERANCE = math.radians(2.0)
SETTLE_TIME = 0.25
# Beyond the settle window, accept small residual error but abort if stuck.
MAX_WAYPOINT_ERROR = math.radians(8.0)
# Joint 6 is vertical with this gripper mount, so an error there only turns
# the box about its vertical axis (it stays upright); allow more slack.
MAX_JOINT6_ERROR = math.radians(15.0)
# Largest joint change allowed between two Cartesian samples; a bigger jump
# means IK switched branch and the box would swing.
MAX_IK_STEP = math.radians(5.0)
ACCELERATION_LIMITS = np.array([0.45, 0.45, 0.55, 0.80, 0.80, 1.00])
UPRIGHT_LINK6_ROTATION = forward_kinematics(UPRIGHT_REFERENCE_CONFIGURATION)[:3, :3]


def _duration_message(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


class PickPlaceController(Node):
    """Plan upright pick/place motions and stream them to Gazebo Sim."""

    def __init__(self) -> None:
        super().__init__("gp7_pick_place_controller")
        # Box A centre at (0.5, 0.5, 0) on the floor, as set by the lecturer.
        self.declare_parameter("box_a_x", 0.50)
        self.declare_parameter("box_a_y", 0.50)
        self.declare_parameter("box_a_top_z", 0.20)
        self.declare_parameter("box_b_x", 0.35)
        self.declare_parameter("box_b_y", 0.25)
        self.declare_parameter("box_b_center_z", BOX_B_SIZE[2] / 2.0)
        self.declare_parameter("approach_height", 0.32)
        # Height above Box B for the initial lift and the joint-1 rotation;
        # keeps the arm well above Box A (top at 0.20 m).
        self.declare_parameter("clearance_height", 0.55)
        # Step 1 lifts the arm by rotating joint 3 by this many degrees.
        self.declare_parameter("joint3_lift_deg", 30.0)
        self.declare_parameter("place_approach_height", 0.32)
        self.declare_parameter("lift_height", 0.32)
        self.declare_parameter("speed_scale", SAFE_SPEED_SCALE)
        # Cartesian (tool) speed and acceleration limits for upright moves.
        self.declare_parameter("cartesian_speed", 0.10)
        self.declare_parameter("cartesian_acceleration", 0.20)

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/model/GP7Robot/joint_trajectory", 10
        )
        self.attach_publisher = self.create_publisher(
            Empty, "/gp7/gripper/attach", 10
        )
        self.detach_publisher = self.create_publisher(
            Empty, "/gp7/gripper/detach", 10
        )

        self.current_joint_positions: np.ndarray | None = None
        self.latest_joint_state_names: tuple[str, ...] = ()
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

    # ------------------------------------------------------------------ I/O

    def _joint_state_callback(self, message: JointState) -> None:
        self.latest_joint_state_names = tuple(message.name)
        positions = {
            name.replace("/", "::").split("::")[-1]: position
            for name, position in zip(message.name, message.position)
        }
        if all(name in positions for name in JOINT_NAMES):
            self.current_joint_positions = np.array(
                [positions[name] for name in JOINT_NAMES], dtype=float
            )

    def _wait_for_bridge(self) -> bool:
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.trajectory_publisher.get_subscription_count() > 0
                and self.detach_publisher.get_subscription_count() > 0
                and self.current_joint_positions is not None
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.trajectory_publisher.get_subscription_count() == 0:
            self.get_logger().error(
                "Gazebo bridge did not subscribe to the joint trajectory topic"
            )
        else:
            self.get_logger().error(
                "No usable /joint_states received for the GP7 joints; names seen: "
                f"{', '.join(self.latest_joint_state_names) or '(none)'}"
            )
        return False

    def _sim_now(self) -> float:
        """Simulation time in seconds (use_sim_time is set by the launch file).

        Gazebo often runs slower than real time, so all waits use the
        simulation clock that also drives the joint trajectory controller.
        """
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _sleep(self, seconds: float) -> None:
        end_time = self._sim_now() + seconds
        while rclpy.ok() and self._sim_now() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _gripper_signal(
        self, publisher, action: str, repeats: int = 1
    ) -> None:
        self.get_logger().info(f"Gripper {action}")
        for _ in range(repeats):
            publisher.publish(Empty())
            self._sleep(0.25)

    # ------------------------------------------------------------ kinematics

    def _solve(self, xyz: np.ndarray, seed: np.ndarray) -> np.ndarray:
        if horizontal_reach(xyz) > MAX_HORIZONTAL_REACH:
            raise ValueError(
                f"Target ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m is "
                f"{horizontal_reach(xyz):.3f} m from the base; GP7 maximum "
                f"horizontal reach is {MAX_HORIZONTAL_REACH:.3f} m"
            )
        target = pose(xyz, UPRIGHT_LINK6_ROTATION)
        return inverse_kinematics(target, seed)

    # ----------------------------------------------------------- trajectories

    def _joint_trajectory(
        self, start: np.ndarray, goal: np.ndarray
    ) -> list[TrajectorySample]:
        """Synchronised trapezoidal joint-space profile (box not held)."""
        scale = float(self.get_parameter("speed_scale").value)
        if not 0.0 < scale <= 1.0:
            raise ValueError("speed_scale must be greater than 0 and at most 1")
        return trapezoidal_segment(
            start,
            goal,
            MAX_JOINT_SPEEDS * scale,
            ACCELERATION_LIMITS,
            sample_period=SAMPLE_PERIOD,
        )

    def _cartesian_trajectory(
        self, q_start: np.ndarray, p_start: np.ndarray, p_goal: np.ndarray
    ) -> list[TrajectorySample]:
        """Straight-line, constant-orientation path with a trapezoidal profile.

        The scalar path length s(t) follows a trapezoidal velocity profile in
        Cartesian space; IK is solved at every sample (seeded by the previous
        solution) with the upright Link6 orientation, so the box never tilts.
        Joint velocities are the finite differences of the IK solutions. If
        any joint would exceed its speed limit the whole profile is slowed.
        """
        length = float(np.linalg.norm(p_goal - p_start))
        if length < 1.0e-6:
            return [
                TrajectorySample(0.0, q_start.copy(), np.zeros(6)),
                TrajectorySample(SAMPLE_PERIOD, q_start.copy(), np.zeros(6)),
            ]
        direction = (p_goal - p_start) / length
        speed = float(self.get_parameter("cartesian_speed").value)
        accel = float(self.get_parameter("cartesian_acceleration").value)
        joint_speed_limits = MAX_JOINT_SPEEDS * float(
            self.get_parameter("speed_scale").value
        )

        for _ in range(6):
            profile = trapezoidal_segment(
                [0.0], [length], [speed], [accel], sample_period=SAMPLE_PERIOD
            )
            times = np.array([sample.time_from_start for sample in profile])
            distances = np.array([sample.positions[0] for sample in profile])

            q = q_start.copy()
            joints = [q.copy()]
            for distance in distances[1:]:
                q_next = self._solve(p_start + direction * distance, q)
                if np.max(np.abs(q_next - q)) > MAX_IK_STEP:
                    raise ValueError(
                        "IK branch switch on the upright Cartesian path at "
                        f"{(p_start + direction * distance).round(3)}"
                    )
                q = q_next
                joints.append(q.copy())
            joints_array = np.array(joints)

            velocities = np.gradient(joints_array, times, axis=0)
            velocities[0] = 0.0
            velocities[-1] = 0.0
            ratio = float(
                np.max(np.max(np.abs(velocities), axis=0) / joint_speed_limits)
            )
            if ratio <= 1.0:
                return [
                    TrajectorySample(float(t), pos, vel)
                    for t, pos, vel in zip(times, joints_array, velocities)
                ]
            speed /= ratio * 1.05
            accel /= ratio * 1.05

        raise ValueError("Could not respect joint speed limits on Cartesian path")

    def _execute(
        self, samples: list[TrajectorySample], goal: np.ndarray, label: str
    ) -> np.ndarray:
        """Publish a trajectory and wait until Gazebo reaches its end point."""
        message = JointTrajectory()
        message.joint_names = list(JOINT_NAMES)
        message.header.stamp = self.get_clock().now().to_msg()
        for sample in samples:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in sample.positions]
            point.velocities = [float(v) for v in sample.velocities]
            point.time_from_start = _duration_message(sample.time_from_start)
            message.points.append(point)
        duration = samples[-1].time_from_start

        self.get_logger().info(
            f"Moving: {label} ({duration:.2f} s trapezoidal profile)"
        )
        self.trajectory_publisher.publish(message)

        deadline = self._sim_now() + duration + 3.0
        settled_since: float | None = None
        last_error = math.inf
        while rclpy.ok() and self._sim_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.current_joint_positions is None:
                continue
            last_error = float(
                np.max(np.abs(self.current_joint_positions - goal))
            )
            if last_error <= WAYPOINT_TOLERANCE:
                if settled_since is None:
                    settled_since = self._sim_now()
                elif self._sim_now() - settled_since >= SETTLE_TIME:
                    return goal
            else:
                settled_since = None

        error = np.abs(self.current_joint_positions - goal)
        if (
            float(np.max(error[:5])) <= MAX_WAYPOINT_ERROR
            and float(error[5]) <= MAX_JOINT6_ERROR
        ):
            self.get_logger().warning(
                f"{label}: finished {math.degrees(last_error):.1f} degrees "
                "from the target; continuing. Error per joint (deg): "
                f"{np.rad2deg(self.current_joint_positions - goal).round(2)}"
            )
            return goal
        raise ValueError(
            f"Gazebo did not reach the {label} waypoint; maximum joint error "
            f"was {math.degrees(last_error):.1f} degrees. Goal (deg) "
            f"{np.rad2deg(goal).round(1)}, actual (deg) "
            f"{np.rad2deg(self.current_joint_positions).round(1)}"
        )

    def _move_joint(
        self, start: np.ndarray, goal: np.ndarray, label: str
    ) -> np.ndarray:
        return self._execute(self._joint_trajectory(start, goal), goal, label)

    def _move_line(
        self,
        q_start: np.ndarray,
        p_start: np.ndarray,
        p_goal: np.ndarray,
        label: str,
    ) -> np.ndarray:
        samples = self._cartesian_trajectory(q_start, p_start, p_goal)
        goal = samples[-1].positions
        return self._execute(samples, goal, f"{label} (upright line)")

    # ------------------------------------------------------------------ task

    def run_demo(self) -> None:
        if not self._wait_for_bridge():
            return

        self._gripper_signal(
            self.detach_publisher, "initial release", repeats=3
        )

        box_a_x = float(self.get_parameter("box_a_x").value)
        box_a_y = float(self.get_parameter("box_a_y").value)
        box_a_top_z = float(self.get_parameter("box_a_top_z").value)
        box_b = np.array(
            [
                float(self.get_parameter("box_b_x").value),
                float(self.get_parameter("box_b_y").value),
                float(self.get_parameter("box_b_center_z").value),
            ],
            dtype=float,
        )
        approach_height = float(self.get_parameter("approach_height").value)
        place_approach_height = float(
            self.get_parameter("place_approach_height").value
        )
        lift_height = float(self.get_parameter("lift_height").value)

        # Link6 is 75 mm above the detachable gripper frame / Box B centre.
        flange_offset = np.array([0.0, 0.0, GRIPPER_LINK_OFFSET])
        pick_hover = box_b + np.array([0.0, 0.0, approach_height]) + flange_offset
        pick_pose = box_b + flange_offset
        lift_pose = box_b + np.array([0.0, 0.0, lift_height]) + flange_offset

        destination = np.array(
            [box_a_x, box_a_y, box_a_top_z + BOX_B_SIZE[2] / 2.0], dtype=float
        )
        place_hover = (
            destination
            + np.array([0.0, 0.0, place_approach_height])
            + flange_offset
        )
        place_pose = destination + flange_offset

        self.get_logger().info(
            f"Box B start ({box_b[0]:.3f}, {box_b[1]:.3f}, {box_b[2]:.3f}) m"
        )
        self.get_logger().info(
            f"Box B target on Box A ({destination[0]:.3f}, "
            f"{destination[1]:.3f}, {destination[2]:.3f}) m"
        )

        start = self.current_joint_positions.copy()
        clearance_height = float(self.get_parameter("clearance_height").value)
        clear_pose = box_b + np.array([0.0, 0.0, clearance_height]) + flange_offset

        # IK (kinematics.py) for a point high above Box B with the gripper
        # pointing straight down. Seeded from the reference pose so the arm
        # reaches over with the forearm coming down onto the box.
        q_clear = self._solve(clear_pose, UPRIGHT_REFERENCE_CONFIGURATION)
        # Check the straight-line descent is solvable before moving.
        self._cartesian_trajectory(q_clear, clear_pose, pick_pose)
        self.get_logger().info(
            f"Pose above Box B, joints (deg): {np.rad2deg(q_clear).round(1)}"
        )

        # 1) Lift the arm: swing joint 3 (with the wrist joints) up into the
        #    arm shape of q_clear while joint 1 stays at home. The arm stays
        #    high, well above Box A.
        q_lift = q_clear.copy()
        q_lift[0] = start[0]
        q = self._move_joint(start, q_lift, "lift arm (joint 3)")

        # 2) Rotate only joint 1 until the arm faces Box B; the gripper ends
        #    directly above the box.
        self.get_logger().info(
            f"Rotating joint 1 to {math.degrees(q_clear[0]):.1f} deg to face Box B"
        )
        q = self._move_joint(q, q_clear, "rotate joint 1 to face Box B")

        # 3) Path planning to the box: straight vertical line, gripper kept
        #    upright, trapezoidal speed, IK solved at every sample.
        q = self._move_line(q, clear_pose, pick_pose, "lower to Box B")

        # 4) Grab Box B.
        self._gripper_signal(self.attach_publisher, "attach", repeats=3)
        self.get_logger().info("Box B grabbed")

        # 5) Raise Box B: straight vertical line back up to the high pose,
        #    gripper (and box) kept upright, then stop and hold.
        q = self._move_line(q, pick_pose, clear_pose, "raise Box B")
        self.get_logger().info("Box B raised")

        # 6) Carry Box B to the centre of Box A. Plan first: a horizontal
        #    straight line at the raised height to above Box A's centre, then
        #    a vertical line down until Box B sits on Box A's top surface.
        above_box_a = np.array([destination[0], destination[1], clear_pose[2]])
        # Release Box B 5 mm above Box A's top so the arm never presses the
        # box into Box A; it settles onto the surface under gravity.
        release_pose = place_pose + np.array([0.0, 0.0, 0.005])
        q_check = self._cartesian_trajectory(q, clear_pose, above_box_a)[-1].positions
        self._cartesian_trajectory(q_check, above_box_a, release_pose)
        self.get_logger().info("Path to the centre of Box A planned and validated")

        q = self._move_line(q, clear_pose, above_box_a, "carry Box B above Box A")
        q = self._move_line(
            q, above_box_a, release_pose, "lower Box B vertically onto Box A"
        )

        # 7) Release Box B on Box A, then lift the empty gripper straight up
        #    and stop.
        self._gripper_signal(self.detach_publisher, "release", repeats=3)
        self.get_logger().info("Box B placed at the centre of Box A")
        self._move_line(q, release_pose, above_box_a, "retreat from Box A")
        self.get_logger().info("Done; stopping here")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceController()
    try:
        node.run_demo()
    except (ValueError, np.linalg.LinAlgError) as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
