import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from geo_gp_fusion.policies.nash_blending import (  # noqa: E402
    nash_blend_pose,
    solve_pose_nash_game,
)


def make_pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_skill_gate_forces_leader_increment():
    result = solve_pose_nash_game(
        leader_increment=(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        gp_increment=(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        c_net=0.1,
        c_gp=0.9,
        g_skill=0.0,
    )

    assert result.case == 'skill_gate'
    assert result.human_action == pytest.approx((1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert result.gp_action == pytest.approx((0.0,) * 6)


def test_high_gp_confidence_moves_fused_pose_toward_gp_target():
    current = make_pose()
    leader = make_pose(x=1.0)
    predicted = make_pose(x=-1.0)

    fused, result = nash_blend_pose(
        current,
        predicted,
        leader,
        c_net=0.1,
        c_gp=0.9,
        human_agreement=0.0,
        gp_agreement=0.0,
    )

    assert result.case == 'nash'
    assert fused.position.x < 0.0


def test_skill_gate_returns_leader_pose():
    current = make_pose()
    leader = make_pose(x=1.0)
    predicted = make_pose(x=-1.0)

    fused, result = nash_blend_pose(
        current,
        predicted,
        leader,
        c_net=0.1,
        c_gp=0.9,
        g_skill=0.0,
    )

    assert result.case == 'skill_gate'
    assert fused.position.x == pytest.approx(leader.position.x)


def test_invalid_increment_dimension_is_rejected():
    with pytest.raises(ValueError):
        solve_pose_nash_game(
            leader_increment=(0.0,) * 5,
            gp_increment=(0.0,) * 6,
            c_net=0.5,
            c_gp=0.5,
        )
