import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from geo_gp_fusion.policies.optimal_blending import (  # noqa: E402
    compute_gp_confidence,
    optimal_prediction_weight,
    pose_conflict,
    pose_orientation_conflict,
    pose_position_conflict,
    solve_scalar_arbitration,
)


def make_pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_human_dominant_confidence_prefers_low_gp_authority():
    result = solve_scalar_arbitration(
        c_net=0.9,
        c_gp=0.1,
        alpha_prev=0.2,
        d=0.0,
        lambda_s=0.0,
        lambda_c=0.0,
    )

    assert result.case == 'interior'
    assert result.alpha_g == pytest.approx(0.1)
    assert result.alpha_h == pytest.approx(0.9)
    assert result.A == pytest.approx(1.0)


def test_gp_dominant_confidence_prefers_high_gp_authority():
    result = solve_scalar_arbitration(
        c_net=0.1,
        c_gp=0.9,
        alpha_prev=0.2,
        d=0.0,
        lambda_s=0.0,
        lambda_c=0.0,
    )

    assert result.case == 'interior'
    assert result.alpha_g == pytest.approx(0.9)
    assert result.alpha_h == pytest.approx(0.1)


def test_high_conflict_uses_boundary_case_with_small_denominator():
    result = solve_scalar_arbitration(
        c_net=0.8,
        c_gp=0.2,
        alpha_prev=0.1,
        d=2.0,
        lambda_s=0.0,
        lambda_c=1.0,
        eps=1e-9,
    )

    assert result.case == 'boundary'
    assert result.A < 0.0
    assert result.alpha_g == pytest.approx(0.0)
    assert result.alpha_h == pytest.approx(1.0)


def test_skill_gate_forces_human_authority():
    result = solve_scalar_arbitration(
        c_net=0.1,
        c_gp=0.9,
        alpha_prev=0.9,
        d=0.0,
        lambda_s=0.0,
        lambda_c=0.0,
        g_skill=0.0,
    )

    assert result.case == 'skill_gate'
    assert result.alpha_g == pytest.approx(0.0)
    assert result.alpha_h == pytest.approx(1.0)


def test_gp_confidence_returns_zero_when_skill_below_gate():
    c_gp, g_skill = compute_gp_confidence(
        skill_confidence=0.2,
        prediction_confidence=1.0,
        point_variance=0.0,
        chunk_error=0.0,
        progress=1.0,
        gp_skill_min=0.5,
    )

    assert g_skill == pytest.approx(0.0)
    assert c_gp == pytest.approx(0.0)


def test_pose_position_conflict_uses_normalized_distance():
    leader = make_pose(x=0.0, y=0.0, z=0.0)
    predicted = make_pose(x=0.3, y=0.4, z=0.0)

    assert pose_position_conflict(leader, predicted, scale=0.5) == pytest.approx(1.0)


def test_pose_orientation_conflict_uses_shortest_quaternion_angle():
    leader = make_pose(qw=1.0)
    predicted = make_pose(qz=1.0, qw=0.0)

    assert pose_orientation_conflict(leader, predicted) == pytest.approx(1.0)


def test_pose_conflict_combines_position_and_orientation():
    leader = make_pose(x=0.0, qz=0.0, qw=1.0)
    predicted = make_pose(x=1.0, qz=1.0, qw=0.0)

    conflict = pose_conflict(
        leader,
        predicted,
        position_scale=1.0,
        orientation_scale=math.pi,
        position_weight=1.0,
        orientation_weight=1.0,
    )

    assert conflict == pytest.approx(1.0)


def test_optimal_prediction_weight_applies_confidence_gain_and_bounds():
    result = optimal_prediction_weight(
        c_net=0.1,
        c_gp=0.4,
        alpha_prev=0.0,
        d=0.0,
        confidence_gain=2.0,
        max_prediction_weight=0.75,
    )

    assert result.alpha_g == pytest.approx(0.8 / 1.1)
    assert result.alpha_h == pytest.approx(0.3 / 1.1)


def test_invalid_nonfinite_input_raises_value_error():
    with pytest.raises(ValueError):
        solve_scalar_arbitration(
            c_net=math.nan,
            c_gp=0.5,
            alpha_prev=0.0,
            d=0.0,
        )
