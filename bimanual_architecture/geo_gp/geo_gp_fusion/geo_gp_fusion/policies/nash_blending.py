"""Blend Geo-GP poses with a position-level non-zero-sum Nash game.

The policy treats the human and GP target poses as player preferences. Each
player contributes a local six-dimensional pose increment, and their
simultaneous best responses form a one-step linear-quadratic Nash equilibrium.
The public result remains a ROS ``Pose``.
"""

from dataclasses import dataclass
import copy
import math

def clamp(value, low, high):
    """Clamp a value to a closed interval.

    Args:
        value: Value to clamp.
        low: Inclusive lower bound.
        high: Inclusive upper bound.

    Returns:
        The clamped value.
    """
    return max(low, min(high, value))


def normalize_quaternion_xyzw(q):
    """Normalize a ROS-order quaternion.

    Args:
        q: Quaternion ordered as ``(x, y, z, w)``.

    Returns:
        A unit quaternion, or the identity quaternion when ``q`` is invalid.
    """
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


@dataclass(frozen=True)
class NashBlendResult:
    """Diagnostics returned with a Nash-blended pose.

    Attributes:
        human_action: Six-dimensional action chosen by the human player.
        gp_action: Six-dimensional action chosen by the GP player.
        q_h: Human confidence weight.
        q_g: GP confidence weight.
        case: Solver outcome, such as ``"nash"`` or ``"human_fallback"``.
    """

    human_action: tuple
    gp_action: tuple
    q_h: float
    q_g: float
    case: str


def _finite(name, value):
    """Convert a value to a finite float.

    Args:
        name: Parameter name used in validation errors.
        value: Value to convert.

    Returns:
        The finite floating-point value.

    Raises:
        ValueError: If ``value`` cannot be converted to a finite float.
    """
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return value


def _quaternion_multiply(q0, q1):
    """Multiply two ROS-order quaternions.

    Args:
        q0: Left quaternion, ordered as ``(x, y, z, w)``.
        q1: Right quaternion, ordered as ``(x, y, z, w)``.

    Returns:
        Product quaternion in ``(x, y, z, w)`` order.
    """
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return (
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    )


def _quaternion_conjugate(q):
    """Return a quaternion conjugate.

    Args:
        q: Quaternion ordered as ``(x, y, z, w)``.

    Returns:
        Conjugate quaternion in ``(x, y, z, w)`` order.
    """
    return (-q[0], -q[1], -q[2], q[3])


def _rotation_vector_between(q_from, q_to):
    """Compute the shortest local rotation vector between two quaternions.

    Args:
        q_from: Starting quaternion in ``(x, y, z, w)`` order.
        q_to: Target quaternion in ``(x, y, z, w)`` order.

    Returns:
        Three-dimensional local rotation vector from ``q_from`` to ``q_to``.
    """
    relative = normalize_quaternion_xyzw(
        _quaternion_multiply(_quaternion_conjugate(q_from), q_to)
    )
    if relative[3] < 0.0:
        relative = tuple(-value for value in relative)
    sin_half = math.sqrt(sum(value * value for value in relative[:3]))
    if sin_half < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(sin_half, clamp(relative[3], -1.0, 1.0))
    scale = angle / sin_half
    return tuple(scale * value for value in relative[:3])


def _quaternion_from_rotation_vector(rotation):
    """Convert a rotation vector to a ROS-order quaternion.

    Args:
        rotation: Three-dimensional rotation vector.

    Returns:
        Quaternion ordered as ``(x, y, z, w)``.
    """
    angle = math.sqrt(sum(value * value for value in rotation))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(0.5 * angle) / angle
    return (
        rotation[0] * scale,
        rotation[1] * scale,
        rotation[2] * scale,
        math.cos(0.5 * angle),
    )


def _pose_increment(current_pose, target_pose):
    """Compute a six-dimensional local increment between poses.

    Args:
        current_pose: Starting pose-like object.
        target_pose: Target pose-like object.

    Returns:
        Translation followed by the local rotation vector.
    """
    current_q = normalize_quaternion_xyzw((
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w,
    ))
    target_q = normalize_quaternion_xyzw((
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w,
    ))
    return (
        float(target_pose.position.x - current_pose.position.x),
        float(target_pose.position.y - current_pose.position.y),
        float(target_pose.position.z - current_pose.position.z),
        *_rotation_vector_between(current_q, target_q),
    )


def _apply_increment(current_pose, increment):
    """Apply a six-dimensional local increment to a pose.

    A deep copy of the supplied message-like object is used rather than
    constructing a ROS ``Pose`` directly. This preserves unit-testability and
    returns a ``geometry_msgs/Pose`` when invoked by the ROS node.

    Args:
        current_pose: Starting pose-like object.
        increment: Translation and local rotation-vector increment.

    Returns:
        Copy of ``current_pose`` updated by ``increment``.
    """
    pose = copy.deepcopy(current_pose)
    pose.position.x = current_pose.position.x + increment[0]
    pose.position.y = current_pose.position.y + increment[1]
    pose.position.z = current_pose.position.z + increment[2]
    current_q = normalize_quaternion_xyzw((
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w,
    ))
    q = normalize_quaternion_xyzw(_quaternion_multiply(
        current_q,
        _quaternion_from_rotation_vector(increment[3:]),
    ))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def solve_pose_nash_game(
    *,
    leader_increment,
    gp_increment,
    c_net,
    c_gp,
    g_skill=1.0,
    confidence_gain=1.0,
    human_effort=0.2,
    gp_effort=0.5,
    human_agreement=0.05,
    gp_agreement=0.10,
    agreement_ratio=0.7,
    rotation_weight=1.0,
    eps=1e-9,
):
    """Solve a one-step six-dimensional linear-quadratic Nash game.

    The state update is ``x_next = x + u_h + u_g``. The two player costs are

    ``J_h = ||x_next - x_h||_Qh^2 + r_h||u_h-d_h||^2
             + w_h||u_h-u_g||^2``

    ``J_g = ||x_next - x_g||_Qg^2 + r_g||u_g||^2
             + w_g||u_g-eta*u_h||^2``.

    ``Q_h`` and ``Q_g`` reuse the existing weighted-blending confidence logic:
    ``Q_h = c_net * D`` and ``Q_g = confidence_gain * c_gp * D``. The GP
    confidence is expected to already include its skill gate and
    variance/error/progress penalties.

    Args:
        leader_increment: Human target increment with six components.
        gp_increment: GP target increment with six components.
        c_net: Human confidence weight before per-dimension scaling.
        c_gp: GP confidence weight before gain and per-dimension scaling.
        g_skill: GP skill-gate value; non-positive values disable GP action.
        confidence_gain: Non-negative multiplier for GP confidence.
        human_effort: Human effort regularization weight.
        gp_effort: GP effort regularization weight.
        human_agreement: Human agreement penalty weight.
        gp_agreement: GP agreement penalty weight.
        agreement_ratio: Fraction of the human action used in GP agreement.
        rotation_weight: Relative confidence weight for rotation dimensions.
        eps: Determinant threshold for detecting a degenerate game.

    Returns:
        Solver diagnostics and the players' equilibrium actions.

    Raises:
        ValueError: If an increment is not six-dimensional or an input is not
            finite.
    """
    if len(leader_increment) != 6 or len(gp_increment) != 6:
        raise ValueError('leader_increment and gp_increment must have length 6')
    d_h = tuple(_finite('leader_increment', value) for value in leader_increment)
    d_g = tuple(_finite('gp_increment', value) for value in gp_increment)
    q_h = clamp(_finite('c_net', c_net), 0.0, 1.0)
    q_g = max(0.0, _finite('c_gp', c_gp)) * max(
        0.0, _finite('confidence_gain', confidence_gain)
    )
    gate = _finite('g_skill', g_skill) > 0.0
    if not gate:
        return NashBlendResult(d_h, (0.0,) * 6, q_h, 0.0, 'skill_gate')

    r_h = max(0.0, _finite('human_effort', human_effort))
    r_g = max(0.0, _finite('gp_effort', gp_effort))
    w_h = max(0.0, _finite('human_agreement', human_agreement))
    w_g = max(0.0, _finite('gp_agreement', gp_agreement))
    eta = clamp(_finite('agreement_ratio', agreement_ratio), 0.0, 1.0)
    rot = max(0.0, _finite('rotation_weight', rotation_weight))
    threshold = max(0.0, _finite('eps', eps))

    human_action = []
    gp_action = []
    for index, (leader_value, gp_value) in enumerate(zip(d_h, d_g)):
        dimension_weight = 1.0 if index < 3 else rot
        qh = q_h * dimension_weight
        qg = q_g * dimension_weight
        a11 = qh + r_h + w_h
        a12 = qh - w_h
        a21 = qg - eta * w_g
        a22 = qg + r_g + w_g
        b1 = (qh + r_h) * leader_value
        b2 = qg * gp_value
        determinant = a11 * a22 - a12 * a21
        if abs(determinant) <= threshold:
            return NashBlendResult(d_h, (0.0,) * 6, q_h, q_g, 'human_fallback')
        human_action.append((b1 * a22 - a12 * b2) / determinant)
        gp_action.append((a11 * b2 - b1 * a21) / determinant)
    return NashBlendResult(tuple(human_action), tuple(gp_action), q_h, q_g, 'nash')


def nash_blend_pose(
    current_pose,
    predicted_pose,
    leader_pose,
    *,
    c_net,
    c_gp,
    g_skill=1.0,
    confidence_gain=1.0,
    human_effort=0.2,
    gp_effort=0.5,
    human_agreement=0.05,
    gp_agreement=0.10,
    agreement_ratio=0.7,
    rotation_weight=1.0,
    eps=1e-9,
):
    """Fuse leader and GP target poses with a one-step Nash equilibrium.

    ``current_pose`` must be the latest executed or last published fused pose;
    it defines the local frame in which pose increments are composed. Missing
    targets preserve weighted-blending fallback behavior.

    Args:
        current_pose: Latest executed or published fused pose.
        predicted_pose: GP target pose, or ``None``.
        leader_pose: Human target pose, or ``None``.
        c_net: Human confidence weight.
        c_gp: GP confidence weight.
        g_skill: GP skill-gate value.
        confidence_gain: Multiplier for GP confidence.
        human_effort: Human effort regularization weight.
        gp_effort: GP effort regularization weight.
        human_agreement: Human agreement penalty weight.
        gp_agreement: GP agreement penalty weight.
        agreement_ratio: Fraction of human action used in GP agreement.
        rotation_weight: Relative confidence weight for rotations.
        eps: Degenerate-game determinant threshold.

    Returns:
        Tuple of the fused pose (or fallback pose) and diagnostics. Diagnostics
        are ``None`` when either target pose is missing.

    Raises:
        ValueError: If both targets exist but ``current_pose`` is ``None``.
    """
    if leader_pose is None:
        return copy.deepcopy(predicted_pose) if predicted_pose is not None else None, None
    if predicted_pose is None:
        return copy.deepcopy(leader_pose), None
    if current_pose is None:
        raise ValueError('current_pose is required when both target poses exist')

    result = solve_pose_nash_game(
        leader_increment=_pose_increment(current_pose, leader_pose),
        gp_increment=_pose_increment(current_pose, predicted_pose),
        c_net=c_net,
        c_gp=c_gp,
        g_skill=g_skill,
        confidence_gain=confidence_gain,
        human_effort=human_effort,
        gp_effort=gp_effort,
        human_agreement=human_agreement,
        gp_agreement=gp_agreement,
        agreement_ratio=agreement_ratio,
        rotation_weight=rotation_weight,
        eps=eps,
    )
    increment = tuple(
        human + gp for human, gp in zip(result.human_action, result.gp_action)
    )
    return _apply_increment(current_pose, increment), result
