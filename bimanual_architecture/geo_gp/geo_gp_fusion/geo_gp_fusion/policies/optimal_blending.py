"""Optimal Human Leader-GP pose blending through scalar arbitration."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ArbitrationResult:
    """Result returned by the scalar arbitration solver.

    Attributes:
        alpha_h: Human leader authority, equal to ``1 - alpha_g``.
        alpha_g: GP autonomy authority clipped to ``[0, 1]``.
        A: Quadratic coefficient used to decide the interior or boundary case.
        case: Solver branch used for the result. Expected values are
            ``"interior"``, ``"boundary"``, and ``"skill_gate"``.
    """

    alpha_h: float
    alpha_g: float
    A: float
    case: str


def clamp(value, low, high):
    """Clamp a numeric value to a closed interval.

    Args:
        value: Value to clamp.
        low: Lower interval bound.
        high: Upper interval bound.

    Returns:
        ``value`` clipped to ``[low, high]``.
    """
    return max(low, min(high, value))


def _finite_float(name, value):
    """Convert an input to a finite float.

    Args:
        name: Input name used in the validation error.
        value: Numeric value.

    Returns:
        A finite float.

    Raises:
        ValueError: If the value cannot be converted to a finite float.
    """
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(out):
        raise ValueError(f'{name} must be a finite number')
    return out


def _pose_quaternion_xyzw(pose):
    """Read a ROS-order quaternion from a pose-like object.

    Args:
        pose: Object with an ``orientation`` field containing ``x, y, z, w``.

    Returns:
        Quaternion tuple in ``(x, y, z, w)`` order.
    """
    orientation = pose.orientation
    return (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )


def _normalize_quaternion_xyzw(q):
    """Normalize a quaternion in ROS ``(x, y, z, w)`` order.

    Args:
        q: Quaternion components in ROS order.

    Returns:
        Normalized quaternion. Invalid or near-zero quaternions fall back to the
        identity rotation.
    """
    norm = math.sqrt(sum(component * component for component in q))
    if norm < 1e-12 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in q)


def quaternion_angular_distance(q0, q1):
    """Compute the shortest angular distance between two quaternions.

    Args:
        q0: First quaternion in ROS ``(x, y, z, w)`` order.
        q1: Second quaternion in ROS ``(x, y, z, w)`` order.

    Returns:
        Shortest rotation angle in radians in ``[0, pi]``.
    """
    q0 = _normalize_quaternion_xyzw(q0)
    q1 = _normalize_quaternion_xyzw(q1)
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    return 2.0 * math.acos(clamp(dot, -1.0, 1.0))


def normalized_weights(*weights):
    """Normalize non-negative weights to sum to one.

    This mirrors the online linear blending confidence helper so that the same
    confidence parameters can feed the optimal arbitration policy.

    Args:
        *weights: Raw non-negative weights. Invalid or negative values are
            treated as zero.

    Returns:
        A list of normalized weights. If the valid sum is too small, returns a
        uniform distribution over the provided weights.
    """
    clean = []
    for weight in weights:
        value = float(weight)
        clean.append(max(0.0, value) if math.isfinite(value) else 0.0)
    total = sum(clean)
    if total <= 1e-12:
        return [1.0 / len(clean)] * len(clean)
    return [weight / total for weight in clean]


def compute_network_confidence(
    delay,
    jitter,
    *,
    network_k_delay=4.5,
    network_delay_max=0.2,
    network_k_jitter=3.0,
    network_jitter_max=0.05,
    network_w_delay=0.5,
    network_w_jitter=0.5,
    network_gamma=1.0,
):
    """Compute leader/network confidence using the online fuser formula.

    Args:
        delay: Latest leader communication delay or round-trip estimate in
            seconds. ``None`` falls back to ``network_delay_max``.
        jitter: Latest delay jitter estimate in seconds. ``None`` falls back to
            ``network_jitter_max``.
        network_k_delay: Exponential decay gain for delay confidence.
        network_delay_max: Delay normalization value in seconds.
        network_k_jitter: Exponential decay gain for jitter confidence.
        network_jitter_max: Jitter normalization value in seconds.
        network_w_delay: Relative weight of delay confidence.
        network_w_jitter: Relative weight of jitter confidence.
        network_gamma: Global exponent applied to network confidence.

    Returns:
        Network confidence ``c_net`` clipped to ``[0, 1]``.
    """
    d_max = max(_finite_float('network_delay_max', network_delay_max), 1e-9)
    j_max = max(_finite_float('network_jitter_max', network_jitter_max), 1e-9)
    delay_value = d_max
    jitter_value = j_max
    if delay is not None and math.isfinite(float(delay)):
        delay_value = max(0.0, float(delay))
    if jitter is not None and math.isfinite(float(jitter)):
        jitter_value = max(0.0, float(jitter))

    c_delay = math.exp(-max(0.0, float(network_k_delay)) * delay_value / d_max)
    c_jitter = math.exp(-max(0.0, float(network_k_jitter)) * jitter_value / j_max)
    w_delay, w_jitter = normalized_weights(network_w_delay, network_w_jitter)
    confidence = (c_delay ** w_delay * c_jitter ** w_jitter) ** max(
        0.0,
        float(network_gamma),
    )
    return clamp(confidence, 0.0, 1.0)


def compute_gp_confidence(
    *,
    skill_confidence,
    prediction_confidence=0.0,
    point_variance=0.0,
    chunk_error=0.0,
    progress=0.0,
    gp_skill_min=0.5,
    gp_k_sigma=2.0,
    gp_k_chunk=1.0,
    gp_error_fail=0.01,
    gp_k_progress=10.0,
    gp_progress_midpoint=0.25,
    gp_w_sigma=0.45,
    gp_w_chunk=0.40,
    gp_w_progress=0.15,
    gp_gamma=1.0,
):
    """Compute GP confidence and binary skill gate.

    The confidence terms match the existing online linear blending policy:
    skill confidence gates the GP, variance and chunk error penalize uncertain
    predictions, and progress raises confidence as the observed prompt becomes
    more informative.

    Args:
        skill_confidence: Confidence of the selected GP skill match.
        prediction_confidence: Fallback confidence when ``skill_confidence`` is
            unavailable.
        point_variance: Variance estimate for the sampled prediction point.
        chunk_error: Recent prediction chunk error.
        progress: Prompt/progression ratio used by the logistic confidence term.
        gp_skill_min: Minimum skill confidence required to enable GP authority.
        gp_k_sigma: Exponential decay gain for variance confidence.
        gp_k_chunk: Exponential decay gain for chunk error confidence.
        gp_error_fail: Chunk error normalization value.
        gp_k_progress: Logistic gain for progress confidence.
        gp_progress_midpoint: Progress value where logistic confidence is 0.5.
        gp_w_sigma: Relative weight of variance confidence.
        gp_w_chunk: Relative weight of chunk-error confidence.
        gp_w_progress: Relative weight of progress confidence.
        gp_gamma: Global exponent applied to GP confidence.

    Returns:
        Tuple ``(c_gp, g_skill)`` where ``c_gp`` is already gated.
    """
    c_skill = skill_confidence
    if c_skill is None or not math.isfinite(float(c_skill)):
        c_skill = prediction_confidence
    if c_skill is None or not math.isfinite(float(c_skill)):
        c_skill = 0.0
    c_skill = clamp(float(c_skill), 0.0, 1.0)
    g_skill = 1.0 if c_skill >= float(gp_skill_min) else 0.0

    variance = max(0.0, _finite_float('point_variance', point_variance))
    error = max(0.0, _finite_float('chunk_error', chunk_error))
    rho = _finite_float('progress', progress)

    c_var = math.exp(-max(0.0, float(gp_k_sigma)) * variance)
    c_chunk = math.exp(
        -max(0.0, float(gp_k_chunk)) * error / max(gp_error_fail, 1e-9)
    )
    c_progress = 1.0 / (
        1.0 + math.exp(-float(gp_k_progress) * (rho - float(gp_progress_midpoint)))
    )
    w_sigma, w_chunk, w_progress = normalized_weights(
        gp_w_sigma,
        gp_w_chunk,
        gp_w_progress,
    )
    confidence = (
        c_var ** w_sigma * c_chunk ** w_chunk * c_progress ** w_progress
    ) ** max(0.0, float(gp_gamma))
    return g_skill * clamp(confidence, 0.0, 1.0), g_skill


def pose_position_conflict(leader_pose, predicted_pose, scale=1.0):
    """Compute scalar human-GP position conflict from two ROS poses.

    Args:
        leader_pose: Human leader command pose.
        predicted_pose: GP predicted command pose.
        scale: Positive normalization scale in meters. Larger values reduce
            the conflict magnitude.

    Returns:
        Euclidean position disagreement divided by ``scale``.
    """
    if leader_pose is None or predicted_pose is None:
        return 0.0
    conflict_scale = max(_finite_float('scale', scale), 1e-12)
    dx = leader_pose.position.x - predicted_pose.position.x
    dy = leader_pose.position.y - predicted_pose.position.y
    dz = leader_pose.position.z - predicted_pose.position.z
    d = math.sqrt(dx * dx + dy * dy + dz * dz) / conflict_scale
    return d if math.isfinite(d) else 0.0


def pose_orientation_conflict(leader_pose, predicted_pose, scale=math.pi):
    """Compute scalar human-GP orientation conflict from two ROS poses.

    Args:
        leader_pose: Human leader command pose.
        predicted_pose: GP predicted command pose.
        scale: Positive normalization scale in radians. The default ``pi`` maps
            the largest possible quaternion disagreement to ``1.0``.

    Returns:
        Shortest quaternion angular distance divided by ``scale``.
    """
    if leader_pose is None or predicted_pose is None:
        return 0.0
    conflict_scale = max(_finite_float('scale', scale), 1e-12)
    angle = quaternion_angular_distance(
        _pose_quaternion_xyzw(leader_pose),
        _pose_quaternion_xyzw(predicted_pose),
    )
    d = angle / conflict_scale
    return d if math.isfinite(d) else 0.0


def pose_conflict(
    leader_pose,
    predicted_pose,
    *,
    position_scale=1.0,
    orientation_scale=math.pi,
    position_weight=1.0,
    orientation_weight=1.0,
):
    """Compute combined position-orientation conflict for two ROS poses.

    The returned scalar is suitable for the arbitration solver's ``d`` input.
    Position and orientation disagreements are normalized separately, then
    combined as a weighted Euclidean norm.

    Args:
        leader_pose: Human leader command pose.
        predicted_pose: GP predicted command pose.
        position_scale: Positive position normalization scale in meters.
        orientation_scale: Positive orientation normalization scale in radians.
        position_weight: Relative weight for normalized position conflict.
        orientation_weight: Relative weight for normalized orientation conflict.

    Returns:
        Combined scalar conflict. Returns zero if either pose is unavailable.
    """
    if leader_pose is None or predicted_pose is None:
        return 0.0
    d_pos = pose_position_conflict(leader_pose, predicted_pose, position_scale)
    d_ori = pose_orientation_conflict(leader_pose, predicted_pose, orientation_scale)
    w_pos, w_ori = normalized_weights(position_weight, orientation_weight)
    d = math.sqrt(w_pos * d_pos * d_pos + w_ori * d_ori * d_ori)
    return d if math.isfinite(d) else 0.0


def solve_scalar_arbitration(
    *,
    c_net,
    c_gp,
    alpha_prev,
    d,
    lambda_s=0.10,
    lambda_c=0.05,
    g_skill=1.0,
    eps=1e-9,
):
    """Solve the real-time scalar arbitration problem for ``alpha_G``.

    Args:
        c_net: Human/network confidence ``R_H``.
        c_gp: GP confidence ``R_G``.
        alpha_prev: Previous GP authority.
        d: Human-GP command conflict.
        lambda_s: Smoothness penalty.
        lambda_c: Conflict penalty.
        g_skill: Binary skill gate. If zero, GP authority is forced to zero.
        eps: Small denominator threshold.

    Returns:
        ``ArbitrationResult(alpha_h, alpha_g, A, case)`` where ``case`` is
        ``"interior"``, ``"boundary"``, or ``"skill_gate"``.
    """
    r_h = clamp(_finite_float('c_net', c_net), 0.0, 1.0)
    r_g = clamp(_finite_float('c_gp', c_gp), 0.0, 1.0)
    previous = clamp(_finite_float('alpha_prev', alpha_prev), 0.0, 1.0)
    conflict = max(0.0, _finite_float('d', d))
    smooth = max(0.0, _finite_float('lambda_s', lambda_s))
    conflict_gain = max(0.0, _finite_float('lambda_c', lambda_c))
    threshold = max(0.0, _finite_float('eps', eps))
    gate = 1.0 if _finite_float('g_skill', g_skill) > 0.0 else 0.0

    d2 = conflict * conflict
    A = r_h + r_g + 2.0 * smooth - 2.0 * conflict_gain * d2

    if A > threshold:
        alpha_hat = (r_g + 2.0 * smooth * previous - conflict_gain * d2) / A
        alpha_g = clamp(alpha_hat, 0.0, 1.0)
        case = 'interior'
    else:
        phi_0 = r_g + 2.0 * smooth * previous * previous
        phi_1 = r_h + 2.0 * smooth * (1.0 - previous) * (1.0 - previous)
        alpha_g = 0.0 if phi_0 <= phi_1 else 1.0
        case = 'boundary'

    if gate == 0.0:
        alpha_g = 0.0
        case = 'skill_gate'

    alpha_h = 1.0 - alpha_g
    return ArbitrationResult(alpha_h=alpha_h, alpha_g=alpha_g, A=A, case=case)


def optimal_prediction_weight(
    *,
    c_net,
    c_gp,
    alpha_prev,
    d,
    lambda_s=0.10,
    lambda_c=0.05,
    g_skill=1.0,
    confidence_gain=1.0,
    min_prediction_weight=0.0,
    max_prediction_weight=1.0,
    authority_eps=1e-9,
):
    """Compute GP authority from existing confidence values.

    This is the drop-in scalar replacement for the linear blending authority
    ``c_gp / (c_net + c_gp + eps)``.

    Args:
        c_net: Human/network confidence ``R_H``.
        c_gp: GP confidence ``R_G`` before optional confidence gain.
        alpha_prev: Previous GP authority.
        d: Human-GP command conflict.
        lambda_s: Smoothness penalty.
        lambda_c: Conflict penalty.
        g_skill: Binary skill gate. If zero, GP authority is forced to zero.
        confidence_gain: Gain applied to ``c_gp`` before solving.
        min_prediction_weight: Lower bound for the returned GP authority.
        max_prediction_weight: Upper bound for the returned GP authority.
        authority_eps: Small denominator threshold passed to the solver.

    Returns:
        Arbitration result after confidence gain and output bounds are applied.
    """
    gain = max(0.0, _finite_float('confidence_gain', confidence_gain))
    c_gp_scaled = clamp(_finite_float('c_gp', c_gp) * gain, 0.0, 1.0)
    result = solve_scalar_arbitration(
        c_net=c_net,
        c_gp=c_gp_scaled,
        alpha_prev=alpha_prev,
        d=d,
        lambda_s=lambda_s,
        lambda_c=lambda_c,
        g_skill=g_skill,
        eps=authority_eps,
    )
    alpha_g = clamp(result.alpha_g, min_prediction_weight, max_prediction_weight)
    return ArbitrationResult(
        alpha_h=1.0 - alpha_g,
        alpha_g=alpha_g,
        A=result.A,
        case=result.case,
    )


def optimal_blend_pose(
    predicted_pose,
    leader_pose,
    *,
    c_net,
    c_gp,
    alpha_prev,
    d,
    lambda_s=0.10,
    lambda_c=0.05,
    g_skill=1.0,
    confidence_gain=1.0,
    min_prediction_weight=0.0,
    max_prediction_weight=1.0,
    authority_eps=1e-9,
):
    """Compute optimal GP authority and apply it to fuse two poses.

    Args:
        predicted_pose: Pose sampled from the GP predicted trajectory.
        leader_pose: Latest human leader target pose.
        c_net: Human/network confidence.
        c_gp: GP confidence before optional gain.
        alpha_prev: Previous GP authority.
        d: Human-GP command conflict.
        lambda_s: Smoothness penalty.
        lambda_c: Conflict penalty.
        g_skill: Binary GP skill gate.
        confidence_gain: Gain applied to GP confidence.
        min_prediction_weight: Lower bound for GP authority.
        max_prediction_weight: Upper bound for GP authority.
        authority_eps: Small solver denominator threshold.

    Returns:
        Tuple ``(fused_pose, arbitration)``. ``fused_pose`` is the pose blended
        with the solved GP authority, and ``arbitration`` contains that authority
        plus the solver branch metadata.
    """
    from geo_gp_fusion.policies.weighted_blending import blend_pose

    arbitration = optimal_prediction_weight(
        c_net=c_net,
        c_gp=c_gp,
        alpha_prev=alpha_prev,
        d=d,
        lambda_s=lambda_s,
        lambda_c=lambda_c,
        g_skill=g_skill,
        confidence_gain=confidence_gain,
        min_prediction_weight=min_prediction_weight,
        max_prediction_weight=max_prediction_weight,
        authority_eps=authority_eps,
    )
    fused_pose = blend_pose(predicted_pose, leader_pose, arbitration.alpha_g)
    return fused_pose, arbitration
