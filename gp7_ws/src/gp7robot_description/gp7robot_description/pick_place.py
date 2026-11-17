"""ROS 2 pick-and-place controller for the GP7 Gazebo demo."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
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
from .trajectory import trapezoidal_segment


MAX_HORIZONTAL_REACH = 0.927
BOX_A_SIZE = (0.30, 0.30, 0.20)
BOX_B_SIZE = (0.05, 0.05, 0.05)
GRIPPER_LINK_OFFSET = 0.075
SAFE_SPEED_SCALE = 0.15
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
    """Plan upright pick/place waypoints and stream them to Gazebo Sim."""

    def __init__(self) -> None:
        super().__init__("gp7_pick_place_controller")
        self.declare_parameter("box_a_x", 0.55)
        self.declare_parameter("box_a_y", 0.55)
        self.declare_parameter("box_a_top_z", 0.20)
        self.declare_parameter("box_b_x", 0.35)
        self.declare_parameter("box_b_y", 0.25)
        self.declare_parameter("box_b_center_z", BOX_B_SIZE[2] / 2.0)
        self.declare_parameter("approach_height", 0.32)
        self.declare_parameter("lift_height", 0.35)
        self.declare_parameter("speed_scale", SAFE_SPEED_SCALE)
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/model/GP7Robot/joint_trajectory", 10
        )
        self.attach_publisher = self.create_publisher(Empty, "/gp7/gripper/attach", 10)
        self.detach_publisher = self.create_publisher(Empty, "/gp7/gripper/detach", 10)

    def _wait_for_bridge(self) -> bool:
        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.trajectory_publisher.get_subscription_count() > 0
                and self.detach_publisher.get_subscription_count() > 0
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error("Gazebo bridge did not subscribe to the joint trajectory topic")
        return False

    def _solve(self, xyz: np.ndarray, seed: np.ndarray) -> np.ndarray:
        if horizontal_reach(xyz) > MAX_HORIZONTAL_REACH:
            raise ValueError(
                f"Target ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m is "
                f"{horizontal_reach(xyz):.3f} m from the base; GP7 maximum "
                f"horizontal reach is {MAX_HORIZONTAL_REACH:.3f} m"
            )
        target = pose(xyz, UPRIGHT_LINK6_ROTATION)
        return inverse_kinematics(target, seed)

    def _make_trajectory(self, start: np.ndarray, goal: np.ndarray) -> tuple[JointTrajectory, float]:
        scale = float(self.get_parameter("speed_scale").value)
        if not 0.0 < scale <= 1.0:
            raise ValueError("speed_scale must be greater than 0 and at most 1")
        samples = trapezoidal_segment(
            start,
            goal,
            MAX_JOINT_SPEEDS * scale,
            ACCELERATION_LIMITS,
            sample_period=0.02,
        )
        message = JointTrajectory()
        message.joint_names = list(JOINT_NAMES)
        message.header.stamp = self.get_clock().now().to_msg()
        for sample in samples:
            point = JointTrajectoryPoint()
            point.positions = sample.positions.tolist()
            point.velocities = sample.velocities.tolist()
            point.time_from_start = _duration_message(sample.time_from_start)
            message.points.append(point)
        return message, samples[-1].time_from_start

    def _move(self, start: np.ndarray, goal: np.ndarray, label: str) -> np.ndarray:
        message, duration = self._make_trajectory(start, goal)
        self.get_logger().info(f"Moving: {label} ({duration:.2f} s trapezoidal profile)")
        self.trajectory_publisher.publish(message)
        end_time = time.monotonic() + duration + 0.25
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)
        return goal

    def _gripper_signal(self, publisher, action: str, repeats: int = 1) -> None:
        self.get_logger().info(f"Gripper {action}")
        for _ in range(repeats):
            publisher.publish(Empty())
            end_time = time.monotonic() + 0.25
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.01)

    def run_demo(self) -> None:
        if not self._wait_for_bridge():
            return

        self._gripper_signal(self.detach_publisher, "initial release", repeats=3)
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
        lift_height = float(self.get_parameter("lift_height").value)
        destination = np.array(
            [box_a_x, box_a_y, box_a_top_z + BOX_B_SIZE[2] / 2.0], dtype=float
        )

        # Link6 is 75 mm above the detachable gripper frame. With the tool
        # held level, this keeps the simple fingers just above the box centre.
        flange_offset = np.array([0.0, 0.0, GRIPPER_LINK_OFFSET])
        pick_hover = box_b + np.array([0.0, 0.0, approach_height]) + flange_offset
        pick_pose = box_b + flange_offset
        lift_pose = box_b + np.array([0.0, 0.0, lift_height]) + flange_offset
        place_hover = destination + np.array([0.0, 0.0, approach_height]) + flange_offset
        place_pose = destination + flange_offset

        self.get_logger().info(
            "Task geometry: Box A top centre "
            f"({box_a_x:.3f}, {box_a_y:.3f}, {box_a_top_z:.3f}) m; "
            f"Box B start ({box_b[0]:.3f}, {box_b[1]:.3f}, {box_b[2]:.3f}) m"
        )
        self.get_logger().info("Box B remains upright; the fixed gripper frame is level throughout")

        start = np.zeros(6, dtype=float)
        q_pick_hover = self._solve(pick_hover, UPRIGHT_REFERENCE_CONFIGURATION)
        q_pick = self._solve(pick_pose, q_pick_hover)
        q_lift = self._solve(lift_pose, q_pick)
        q_place_hover = self._solve(place_hover, q_lift)
        q_place = self._solve(place_pose, q_place_hover)
        q_retreat = self._solve(place_hover, q_place)

        self._move(start, q_pick_hover, "hover above Box B")
        self._move(q_pick_hover, q_pick, "lower to Box B")
        self._gripper_signal(self.attach_publisher, "attach")
        self._move(q_pick, q_lift, "lift Box B")
        self._move(q_lift, q_place_hover, "carry above Box A")
        self._move(q_place_hover, q_place, "lower Box B onto Box A")
        self._gripper_signal(self.detach_publisher, "release")
        self._move(q_place, q_retreat, "retreat")
        self._move(q_retreat, start, "return to home")
        self.get_logger().info("Pick-and-place sequence completed")


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
