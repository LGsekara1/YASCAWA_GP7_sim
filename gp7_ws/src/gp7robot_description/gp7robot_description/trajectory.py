"""Time-synchronised trapezoidal joint trajectory generation for the GP7."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    time_from_start: float
    positions: np.ndarray
    velocities: np.ndarray


def _minimum_duration(distance: float, max_velocity: float, max_acceleration: float) -> float:
    if distance <= 1.0e-12:
        return 0.0
    transition_distance = max_velocity * max_velocity / max_acceleration
    if distance <= transition_distance:
        return 2.0 * math.sqrt(distance / max_acceleration)
    return 2.0 * max_velocity / max_acceleration + (distance - transition_distance) / max_velocity


def trapezoidal_segment(
    start: Iterable[float],
    goal: Iterable[float],
    max_velocities: Iterable[float],
    max_accelerations: Iterable[float],
    *,
    sample_period: float = 0.02,
) -> list[TrajectorySample]:
    """Create a synchronized rest-to-rest trapezoidal trajectory.

    Each axis uses a symmetric accelerate/cruise/decelerate profile. The
    longest axis sets the shared duration, so every joint starts and ends at
    the same time while staying within the requested velocity and acceleration
    limits.
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
    duration = max(
        (_minimum_duration(float(d), float(v), float(a)) for d, v, a in zip(distance, vmax, amax)),
        default=0.0,
    )
    if duration <= 1.0e-9:
        return [TrajectorySample(0.0, q0.copy(), np.zeros_like(q0)), TrajectorySample(0.02, q1.copy(), np.zeros_like(q1))]

    acceleration_times = np.zeros_like(q0)
    peak_velocities = np.zeros_like(q0)
    accelerations = np.zeros_like(q0)
    for index, travel in enumerate(distance):
        if travel <= 1.0e-12:
            continue
        discriminant = max(duration * duration - 4.0 * travel / amax[index], 0.0)
        t_accel = 0.5 * (duration - math.sqrt(discriminant))
        t_accel = max(t_accel, 1.0e-9)
        t_accel = min(t_accel, duration * 0.5)
        peak = travel / (duration - t_accel)
        if peak > vmax[index] * (1.0 + 1.0e-8):
            # Numerical tolerance at the limiting trapezoid duration.
            peak = float(vmax[index])
            t_accel = duration - travel / peak
        acceleration_times[index] = t_accel
        peak_velocities[index] = peak
        accelerations[index] = peak / t_accel

    times = list(np.arange(0.0, duration, sample_period))
    if not times or times[0] != 0.0:
        times.insert(0, 0.0)
    times.append(duration)
    samples: list[TrajectorySample] = []
    direction = np.sign(delta)
    for t in times:
        positions = q0.copy()
        velocities = np.zeros_like(q0)
        for index, travel in enumerate(distance):
            if travel <= 1.0e-12:
                continue
            ta = acceleration_times[index]
            vp = peak_velocities[index]
            accel = accelerations[index]
            if t < ta:
                displacement = 0.5 * accel * t * t
                velocity = accel * t
            elif t < duration - ta:
                displacement = 0.5 * accel * ta * ta + vp * (t - ta)
                velocity = vp
            else:
                remaining = duration - t
                displacement = travel - 0.5 * accel * remaining * remaining
                velocity = accel * remaining
            positions[index] += direction[index] * displacement
            velocities[index] = direction[index] * velocity
        samples.append(TrajectorySample(float(t), positions, velocities))
    samples[-1] = TrajectorySample(duration, q1.copy(), np.zeros_like(q1))
    return samples

