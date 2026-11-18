"""Offline reachability check and random start positions for Box B.

The check replays the planning steps of ``pick_place.PickPlaceController``
without ROS or Gazebo: it reuses the controller's own ``_solve`` and
``_cartesian_path`` methods (same IK, same tolerances, same branch-switch and
joint-speed checks), so a position accepted here is one the demo can plan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .kinematics import UPRIGHT_REFERENCE_CONFIGURATION
from .pick_place import (
    BLEND_RADIUS,
    BOX_B_SIZE,
    CARTESIAN_ACCELERATION,
    CARTESIAN_SPEED,
    GRIPPER_LINK_OFFSET,
    MAX_HORIZONTAL_REACH,
    SAFE_SPEED_SCALE,
    PickPlaceController,
)

# Defaults mirror the parameters declared by PickPlaceController.
DEFAULT_PARAMETERS = {
    "box_a_x": 0.50,
    "box_a_y": 0.50,
    "box_a_top_z": 0.20,
    "box_b_center_z": BOX_B_SIZE[2] / 2.0,
    "clearance_height": 0.55,
    "retreat_height": 0.15,
    "speed_scale": SAFE_SPEED_SCALE,
    "cartesian_speed": CARTESIAN_SPEED,
    "cartesian_acceleration": CARTESIAN_ACCELERATION,
    "blend_radius": BLEND_RADIUS,
}
BOX_A_SIZE = 0.30
# Keep Box B this far (edge to edge) from Box A and inside these radii.
BOX_A_MARGIN = 0.05
MIN_RADIUS = 0.30  # stay clear of the robot base
MAX_RADIUS = MAX_HORIZONTAL_REACH - 0.05


@dataclass
class _Parameter:
    value: float


class _PlanChecker:
    """Just enough of PickPlaceController to run its planning methods."""

    _solve = PickPlaceController._solve
    _cartesian_path = PickPlaceController._cartesian_path
    _cartesian_trajectory = PickPlaceController._cartesian_trajectory

    def __init__(self, parameters: dict[str, float]) -> None:
        self._parameters = parameters

    def get_parameter(self, name: str) -> _Parameter:
        return _Parameter(self._parameters[name])


def overlaps_box_a(x: float, y: float, parameters: dict[str, float] | None = None) -> bool:
    p = {**DEFAULT_PARAMETERS, **(parameters or {})}
    limit = BOX_A_SIZE / 2.0 + BOX_B_SIZE[0] / 2.0 + BOX_A_MARGIN
    return abs(x - p["box_a_x"]) < limit and abs(y - p["box_a_y"]) < limit


def check_box_b(
    x: float, y: float, parameters: dict[str, float] | None = None
) -> str | None:
    """Return None if the pick-and-place plan works for Box B at (x, y),
    otherwise a short reason why it does not."""
    p = {**DEFAULT_PARAMETERS, **(parameters or {})}
    radius = math.hypot(x, y)
    if radius < MIN_RADIUS:
        return f"too close to the robot base ({radius:.2f} m)"
    if radius > MAX_RADIUS:
        return f"outside the reach ({radius:.2f} m)"
    if overlaps_box_a(x, y, p):
        return "on or next to Box A"

    planner = _PlanChecker(p)
    flange = np.array([0.0, 0.0, GRIPPER_LINK_OFFSET])
    box_b = np.array([x, y, p["box_b_center_z"]])
    pick_pose = box_b + flange
    clear_pose = box_b + np.array([0.0, 0.0, p["clearance_height"]]) + flange
    destination = np.array(
        [p["box_a_x"], p["box_a_y"], p["box_a_top_z"] + BOX_B_SIZE[2] / 2.0]
    )
    release_pose = destination + flange + np.array([0.0, 0.0, 0.005])
    above_box_a = np.array([destination[0], destination[1], clear_pose[2]])
    try:
        # Same steps, in the same order, as PickPlaceController.run_demo.
        q_clear = planner._solve(clear_pose, UPRIGHT_REFERENCE_CONFIGURATION)
        q = planner._cartesian_path(q_clear, [clear_pose, pick_pose])[-1].positions
        q = planner._cartesian_path(
            q, [pick_pose, clear_pose, above_box_a, release_pose]
        )[-1].positions
        retreat_pose = release_pose + np.array([0.0, 0.0, p["retreat_height"]])
        planner._cartesian_path(q, [release_pose, retreat_pose])
    except (ValueError, np.linalg.LinAlgError) as error:
        return str(error)
    return None


def random_box_b(
    rng: np.random.Generator | None = None,
    parameters: dict[str, float] | None = None,
    attempts: int = 40,
) -> tuple[float, float]:
    """Pick a random Box B position that passes ``check_box_b``."""
    rng = rng or np.random.default_rng()
    for _ in range(attempts):
        # Uniform over the annulus area, any direction around the base.
        radius = math.sqrt(rng.uniform(MIN_RADIUS ** 2, MAX_RADIUS ** 2))
        angle = rng.uniform(-math.pi, math.pi)
        x, y = round(radius * math.cos(angle), 3), round(radius * math.sin(angle), 3)
        if overlaps_box_a(x, y, parameters):
            continue
        if check_box_b(x, y, parameters) is None:
            return x, y
    raise RuntimeError(f"no reachable Box B position found in {attempts} tries")
