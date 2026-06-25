"""Linear pose blending utilities for online Geo-GP fusion."""

import copy
import math

from geometry_msgs.msg import Pose


def clamp(value, low, high):
    """
    Clamp a numeric value to a closed interval.

    Args:
        value: Input value.
        low: Minimum allowed value.
        high: Maximum allowed value.

    Returns:
        The input clipped to ``[low, high]``.
    """
    return max(low, min(high, value))


def normalize_quaternion_xyzw(q):
    """
    Normalize a quaternion stored as ``(x, y, z, w)``.

    Args:
        q: Iterable with four quaternion components in ROS order.

    Returns:
        A normalized ``(x, y, z, w)`` tuple, or identity if invalid.
    """
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm < 1e-12 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def slerp_xyzw(q0, q1, t):
    """
    Spherically interpolate two ROS-order quaternions.

    Args:
        q0: Start quaternion as ``(x, y, z, w)``.
        q1: Goal quaternion as ``(x, y, z, w)``.
        t: Interpolation ratio where 0 selects ``q0`` and 1 selects ``q1``.

    Returns:
        Interpolated quaternion as a normalized ``(x, y, z, w)`` tuple.
    """
    q0 = normalize_quaternion_xyzw(q0)
    q1 = normalize_quaternion_xyzw(q1)
    t = clamp(t, 0.0, 1.0)

    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-x for x in q1)
        dot = -dot

    if dot > 0.9995:
        out = tuple((1.0 - t) * a + t * b for a, b in zip(q0, q1))
        return normalize_quaternion_xyzw(out)

    theta_0 = math.acos(clamp(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return normalize_quaternion_xyzw(tuple(s0 * a + s1 * b for a, b in zip(q0, q1)))


def interpolate_pose(p0, p1, ratio):
    """
    Interpolate between two poses.

    Args:
        p0: Start pose.
        p1: Goal pose.
        ratio: Interpolation ratio where 0 selects ``p0`` and 1 selects ``p1``.

    Returns:
        A new ``geometry_msgs.msg.Pose`` with linearly interpolated position and
        slerped orientation.
    """
    ratio = clamp(ratio, 0.0, 1.0)
    pose = Pose()
    pose.position.x = (1.0 - ratio) * p0.position.x + ratio * p1.position.x
    pose.position.y = (1.0 - ratio) * p0.position.y + ratio * p1.position.y
    pose.position.z = (1.0 - ratio) * p0.position.z + ratio * p1.position.z

    q = slerp_xyzw(
        (p0.orientation.x, p0.orientation.y, p0.orientation.z, p0.orientation.w),
        (p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w),
        ratio,
    )
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def sample_timed_pose(poses, times, elapsed):
    """
    Sample a pose trajectory at an elapsed time.

    Args:
        poses: Sequence of trajectory poses.
        times: Monotonic seconds-from-start values aligned with ``poses``.
        elapsed: Query time in seconds from trajectory start.

    Returns:
        A copied or interpolated pose, or ``None`` when ``poses`` is empty.
    """
    if not poses:
        return None
    if len(poses) == 1 or not times:
        return copy.deepcopy(poses[0])
    if elapsed <= times[0]:
        return copy.deepcopy(poses[0])
    if elapsed >= times[-1]:
        return copy.deepcopy(poses[-1])

    hi = 1
    while hi < len(times) and times[hi] < elapsed:
        hi += 1
    lo = max(0, hi - 1)

    span = times[hi] - times[lo]
    ratio = 0.0 if abs(span) < 1e-12 else (elapsed - times[lo]) / span
    return interpolate_pose(poses[lo], poses[hi], ratio)


def blend_pose(predicted_pose, leader_pose, prediction_weight):
    """
    Blend prediction and TDPA-integrated leader poses.

    Args:
        predicted_pose: Pose sampled from the predicted trajectory.
        leader_pose: Latest TDPA-integrated leader target pose.
        prediction_weight: Weight for ``predicted_pose`` in ``[0, 1]``.

    Returns:
        A new fused pose. If either input pose is unavailable, returns a copy of
        the available pose; returns ``None`` when both inputs are unavailable.
    """
    if predicted_pose is None:
        return copy.deepcopy(leader_pose) if leader_pose is not None else None
    if leader_pose is None:
        return copy.deepcopy(predicted_pose)

    return interpolate_pose(leader_pose, predicted_pose, clamp(prediction_weight, 0.0, 1.0))
