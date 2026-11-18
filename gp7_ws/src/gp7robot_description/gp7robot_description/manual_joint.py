"""Move one GP7 joint while commanding the other five to hold position."""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from .kinematics import JOINT_LIMITS, JOINT_NAMES
from .trajectory import TrajectorySample, joint_trajectory_message


class ManualJointCommander(Node):
    def __init__(self) -> None:
        super().__init__("gp7_manual_joint_commander")
        self.current_positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._joint_state, 10)
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/model/GP7Robot/joint_trajectory", 10
        )

    def _joint_state(self, message: JointState) -> None:
        self.current_positions.update(zip(message.name, message.position))

    def move(self, joint_name: str, target: float, duration: float, max_step: float) -> None:
        if joint_name not in JOINT_NAMES:
            raise ValueError(f"Unknown joint {joint_name!r}; choose from {', '.join(JOINT_NAMES)}")
        if duration <= 0.0:
            raise ValueError("duration must be positive")
        if max_step <= 0.0:
            raise ValueError("max_step must be positive")

        index = JOINT_NAMES.index(joint_name)
        lower, upper = JOINT_LIMITS[index]
        if not lower <= target <= upper:
            raise ValueError(
                f"{joint_name} target must be between {lower:.3f} and {upper:.3f} rad "
                f"({math.degrees(lower):.1f} and {math.degrees(upper):.1f} deg)"
            )

        deadline = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < deadline:
            ready = all(name in self.current_positions for name in JOINT_NAMES)
            connected = self.trajectory_publisher.get_subscription_count() > 0
            if ready and connected:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if not all(name in self.current_positions for name in JOINT_NAMES):
            raise RuntimeError("No complete /joint_states message arrived from Gazebo")
        if self.trajectory_publisher.get_subscription_count() == 0:
            raise RuntimeError("No Gazebo bridge is listening for joint trajectories")

        start = [float(self.current_positions[name]) for name in JOINT_NAMES]
        difference = target - start[index]
        if abs(difference) > max_step:
            raise ValueError(
                f"This step is {math.degrees(abs(difference)):.1f} deg; "
                f"the safe per-command limit is {math.degrees(max_step):.1f} deg. "
                "Use a nearer target or explicitly raise --max-step."
            )
        goal = start.copy()
        goal[index] = target

        trajectory = joint_trajectory_message(
            JOINT_NAMES,
            [TrajectorySample(duration, np.array(goal), np.zeros(len(JOINT_NAMES)))],
            self.get_clock().now().to_msg(),
        )
        self.trajectory_publisher.publish(trajectory)

        self.get_logger().info(
            f"{joint_name}: {start[index]:.3f} -> {target:.3f} rad "
            f"({math.degrees(start[index]):.1f} -> {math.degrees(target):.1f} deg) "
            f"in {duration:.2f} s; other joints hold position"
        )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Move one GP7 joint and hold the other five")
    parser.add_argument("joint", help=f"joint name: {', '.join(JOINT_NAMES)}")
    parser.add_argument("target_rad", type=float, help="target angle in radians")
    parser.add_argument("--duration", type=float, default=2.0, help="move duration in seconds")
    parser.add_argument("--max-step", type=float, default=0.5, help="maximum move per command in radians")
    options = parser.parse_args(sys.argv[1:])

    rclpy.init(args=[])
    node = ManualJointCommander()
    try:
        node.move(options.joint, options.target_rad, options.duration, options.max_step)
    except (ValueError, RuntimeError) as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

