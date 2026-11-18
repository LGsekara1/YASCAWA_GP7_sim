"""Smooth, time-synchronised joint trajectory generation for the GP7."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class TrajectorySample:
    time_from_start: float
    positions: np.ndarray
    velocities: np.ndarray


def scurve_segment(
    start: Iterable[float],
    goal: Iterable[float],
    max_velocities: Iterable[float],
    max_accelerations: Iterable[float],
    *,
    sample_period: float = 0.02,
) -> list[TrajectorySample]:
    """Create a synchronized rest-to-rest S-curve trajectory.

    During each ramp the acceleration follows a sin^2 pulse, so acceleration
    and jerk are both continuous and start/end at zero (no step changes in
    acceleration like a trapezoid, and no jerk spikes like a cosine ramp).
    The profile is split into thirds: accelerate, cruise, decelerate. All
    joints share one normalised path parameter s(t) in [0, 1], so they start
    and stop together and the joint-space path is unchanged.

    With ramp time Ta = T / 3 the normalised peak velocity is 1.5 / T and the
    peak acceleration is 2 * (1.5 / T) / Ta = 9 / T**2, which gives the
    shortest T that respects every joint's limits.
    """
    q0 = np.asarray(tuple(start), dtype=float)
    q1 = np.asarray(tuple(goal), dtype=float)
    vmax = np.asarray(tuple(max_velocities), dtype=float)
    amax = np.asarray(tuple(max_accelerations), dtype=float)
    if q0.ndim != 1 or q0.shape != q1.shape or q0.shape != vmax.shape or q0.shape != amax.shape:
        raise ValueError("Start, goal, velocity, and acceleration arrays must have the same shape")
    if np.any(vmax <= 0.0) or np.any(amax <= 0.0) or sample_period <= 0.0:
        raise ValueError("Limits and sample period must be positive")

    delta = q1 - q0
    distance = np.abs(delta)
    if float(np.max(distance, initial=0.0)) <= 1.0e-12:
        return [
            TrajectorySample(0.0, q0.copy(), np.zeros_like(q0)),
            TrajectorySample(sample_period, q1.copy(), np.zeros_like(q1)),
        ]

    duration = float(
        np.max(
            np.maximum(
                1.5 * distance / vmax,
                np.sqrt(9.0 * distance / amax),
            )
        )
    )
    ramp = duration / 3.0
    peak = 1.0 / (duration - ramp)  # normalised cruise velocity
    accel_peak = 2.0 * peak / ramp  # peak of the sin^2 acceleration pulse

    def profile(t: float) -> tuple[float, float]:
        """Normalised position s and velocity ds/dt at time t."""
        if t <= 0.0:
            return 0.0, 0.0
        if t >= duration:
            return 1.0, 0.0
        if t < ramp:
            # a(t) = A sin^2(pi t / Ta), integrated twice from rest.
            phase = 2.0 * math.pi * t / ramp
            return (
                accel_peak
                * (t * t / 4.0 + ramp * ramp / (8.0 * math.pi**2) * (math.cos(phase) - 1.0)),
                accel_peak * (t / 2.0 - ramp / (4.0 * math.pi) * math.sin(phase)),
            )
        if t <= duration - ramp:
            return 0.5 * peak * ramp + peak * (t - ramp), peak
        s_mirror, v_mirror = profile(duration - t)
        return 1.0 - s_mirror, v_mirror

    times = list(np.arange(0.0, duration, sample_period))
    times.append(duration)
    samples: list[TrajectorySample] = []
    for t in times:
        s, v = profile(float(t))
        samples.append(TrajectorySample(float(t), q0 + delta * s, delta * v))
    samples[-1] = TrajectorySample(duration, q1.copy(), np.zeros_like(q1))
    return samples


def path_speed_profile(
    length: float,
    max_speed: float,
    max_acceleration: float,
    *,
    sample_period: float = 0.02,
) -> list[TrajectorySample]:
    """Rest-to-rest profile along a path of ``length`` metres.

    Same sin^2 acceleration pulse as ``scurve_segment`` (continuous
    acceleration and jerk), but the ramp time comes from the acceleration
    limit (Ta = 2 v / a) instead of always being a third of the move, so a
    long path cruises at full speed for most of its length rather than
    spending two thirds of the time speeding up and slowing down.
    Positions and velocities are 1-element arrays (distance along the path).
    """
    if length <= 0.0 or max_speed <= 0.0 or max_acceleration <= 0.0:
        raise ValueError("Length, speed and acceleration must be positive")
    speed = max_speed
    ramp = 2.0 * speed / max_acceleration
    if speed * ramp > length:  # too short to reach full speed
        speed = math.sqrt(max_acceleration * length / 2.0)
        ramp = 2.0 * speed / max_acceleration
    cruise = (length - speed * ramp) / speed
    duration = 2.0 * ramp + cruise
    accel_peak = 2.0 * speed / ramp

    def profile(t: float) -> tuple[float, float]:
        if t <= 0.0:
            return 0.0, 0.0
        if t >= duration:
            return length, 0.0
        if t < ramp:
            phase = 2.0 * math.pi * t / ramp
            return (
                accel_peak
                * (t * t / 4.0 + ramp * ramp / (8.0 * math.pi**2) * (math.cos(phase) - 1.0)),
                accel_peak * (t / 2.0 - ramp / (4.0 * math.pi) * math.sin(phase)),
            )
        if t <= ramp + cruise:
            return 0.5 * speed * ramp + speed * (t - ramp), speed
        s_mirror, v_mirror = profile(duration - t)
        return length - s_mirror, v_mirror

    times = list(np.arange(0.0, duration, sample_period))
    times.append(duration)
    samples = []
    for t in times:
        s, v = profile(float(t))
        samples.append(TrajectorySample(float(t), np.array([s]), np.array([v])))
    samples[-1] = TrajectorySample(duration, np.array([length]), np.zeros(1))
    return samples


def superpose(
    segments: list[list[TrajectorySample]],
    start_times: Iterable[float],
    *,
    sample_period: float = 0.02,
) -> list[TrajectorySample]:
    """Blend rest-to-rest segments by overlapping them in time.

    Segment ``i`` starts at ``start_times[i]``; its displacement (relative to
    its own first sample) is added on top of the previous segments. Starting
    the next segment while the previous one is still decelerating removes
    the stop between them, and because every segment is a smooth S-curve the
    sum is smooth too: corners are rounded instead of stopped at. Each
    segment holds still before it starts and after it ends.
    """
    offsets = [float(t) for t in start_times]
    if len(offsets) != len(segments) or not segments:
        raise ValueError("Need one start time per segment")
    base = segments[0][0].positions.copy()
    duration = max(t0 + seg[-1].time_from_start for t0, seg in zip(offsets, segments))

    tables = []
    for seg in segments:
        times = np.array([s.time_from_start for s in seg])
        positions = np.array([s.positions for s in seg])
        velocities = np.array([s.velocities for s in seg])
        tables.append((times, positions - positions[0], velocities))

    def interp(t, times, values):
        return np.array([np.interp(t, times, values[:, k]) for k in range(values.shape[1])])

    times = list(np.arange(0.0, duration, sample_period))
    times.append(duration)
    samples: list[TrajectorySample] = []
    for t in times:
        position = base.copy()
        velocity = np.zeros_like(base)
        for t0, (seg_times, displacement, seg_velocity) in zip(offsets, tables):
            local = t - t0
            if local <= 0.0:
                continue
            position += interp(local, seg_times, displacement)
            if local < seg_times[-1]:
                velocity += interp(local, seg_times, seg_velocity)
        samples.append(TrajectorySample(float(t), position, velocity))
    last = samples[-1]
    samples[-1] = TrajectorySample(last.time_from_start, last.positions, np.zeros_like(base))
    return samples


def joint_trajectory_message(
    joint_names: Iterable[str], samples: list[TrajectorySample], stamp
) -> JointTrajectory:
    """Pack trajectory samples into a ROS JointTrajectory message."""
    message = JointTrajectory()
    message.joint_names = list(joint_names)
    message.header.stamp = stamp
    for sample in samples:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in sample.positions]
        point.velocities = [float(v) for v in sample.velocities]
        nanoseconds = int(round(sample.time_from_start * 1.0e9))
        point.time_from_start = Duration(
            sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000
        )
        message.points.append(point)
    return message
