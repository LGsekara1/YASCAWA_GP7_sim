"""Reset BoxB to its start pose and return the GP7 arm to home."""

from __future__ import annotations

import subprocess
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .kinematics import JOINT_NAMES, MAX_JOINT_SPEEDS
from .trajectory import trapezoidal_segment

ACCELERATION_LIMITS = np.array([0.45, 0.45, 0.55, 0.80, 0.80, 1.00])
SAFE_SPEED_SCALE = 0.20


def _duration_message(seconds: float) -> Duration:
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


class ResetWorldNode(Node):
    """Detach BoxB, teleport it to start, and drive GP7 joints to zero."""

    def __init__(self) -> None:
        super().__init__("gp7_reset_world")
        self.declare_parameter("world_name", "gp7_pick_place")
        self.declare_parameter("box_b_x", 0.35)
        self.declare_parameter("box_b_y", 0.25)
        self.declare_parameter("box_b_center_z", 0.025)

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/model/GP7Robot/joint_trajectory", 10
        )
        self.detach_publisher = self.create_publisher(Empty, "/gp7/gripper/detach", 10)

        self._current_joints: np.ndarray | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(joint in name_to_pos for joint in JOINT_NAMES):
            self._current_joints = np.array(
                [name_to_pos[joint] for joint in JOINT_NAMES], dtype=float
            )

    def _wait_for_ready(self) -> bool:
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.trajectory_publisher.get_subscription_count() > 0
                and self.detach_publisher.get_subscription_count() > 0
                and self._current_joints is not None
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error("Timed out waiting for Gazebo bridge or /joint_states")
        return False

    def _reset_box_b(self) -> None:
        world_name = str(self.get_parameter("world_name").value)
        x = float(self.get_parameter("box_b_x").value)
        y = float(self.get_parameter("box_b_y").value)
        z = float(self.get_parameter("box_b_center_z").value)

        req = (
            f'name: "BoxB", '
            f"position: {{x: {x}, y: {y}, z: {z}}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
        )
        cmd = [
            "gz",
            "service",
            "-s",
            f"/world/{world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            req,
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info(f"BoxB reset to ({x:.3f}, {y:.3f}, {z:.3f}) m")
        else:
            self.get_logger().warning(f"Failed to reset BoxB pose: {result.stderr.strip()}")

    def _home_robot(self) -> None:
        assert self._current_joints is not None
        start = self._current_joints
        home = np.zeros(6, dtype=float)

        if np.allclose(start, home, atol=1e-2):
            self.get_logger().info("Robot is already at home position")
            return

        samples = trapezoidal_segment(
            start,
            home,
            MAX_JOINT_SPEEDS * SAFE_SPEED_SCALE,
            ACCELERATION_LIMITS,
            sample_period=0.12,
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

        duration = samples[-1].time_from_start
        self.get_logger().info(f"Homing GP7 robot ({duration:.2f} s)")
        self.trajectory_publisher.publish(message)

        end_time = time.monotonic() + duration + 0.3
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)

    def reset_world(self) -> None:
        if not self._wait_for_ready():
            return

        # 1. Release gripper first so BoxB isn't dragged while homing
        self.detach_publisher.publish(Empty())
        self.get_logger().info("Published gripper detach")
        end_time = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.02)

        # 2. Teleport BoxB back to its initial coordinates
        self._reset_box_b()

        # 3. Drive the robot arm smoothly back to zero configuration
        self._home_robot()
        self.get_logger().info("World reset complete")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResetWorldNode()
    try:
        node.reset_world()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
