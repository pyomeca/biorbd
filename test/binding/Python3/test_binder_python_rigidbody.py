"""
Test for file IO
"""
from pathlib import Path

import numpy as np
import pytest

import biorbd


def test_load_model():
    biorbd.Model('../../models/pyomecaman.bioMod')


def test_dof_ranges():
    m = biorbd.Model('../../models/pyomecaman.bioMod')
    pi = 3.14159265358979323846

    # Pelvis
    QRanges = m.segment(0).QRanges()
    assert(QRanges[0].min() == -10)
    assert(QRanges[0].max() == 10)
    assert(QRanges[1].min() == -10)
    assert(QRanges[1].max() == 10)
    assert(QRanges[2].min() == -pi)
    assert(QRanges[2].max() == pi)

    # BrasD
    QRanges = m.segment(3).QRanges()
    assert(QRanges[0].min() == -pi)
    assert(QRanges[0].max() == pi)
    assert(QRanges[1].min() == 0)
    assert(QRanges[1].max() == pi)

    # BrasG
    QRanges = m.segment(4).QRanges()
    assert(QRanges[0].min() == -pi)
    assert(QRanges[0].max() == pi)
    assert(QRanges[1].min() == 0)
    assert(QRanges[1].max() == pi)

    # CuisseD
    QRanges = m.segment(5).QRanges()
    assert(QRanges[0].min() == -pi / 12)
    assert(QRanges[0].max() == pi / 2 + pi / 3)

    # JambeD
    QRanges = m.segment(6).QRanges()
    assert(QRanges[0].min() == -pi / 2 - pi / 6)
    assert(QRanges[0].max() == 0)

    # PiedD
    QRanges = m.segment(7).QRanges()
    assert(QRanges[0].min() == -pi / 2)
    assert(QRanges[0].max() == pi / 2)

    # CuisseG
    QRanges = m.segment(8).QRanges()
    assert(QRanges[0].min() == -pi / 12)
    assert(QRanges[0].max() == pi / 2 + pi / 3)

    # JambeG
    QRanges = m.segment(9).QRanges()
    assert(QRanges[0].min() == -pi / 2 - pi / 6)
    assert(QRanges[0].max() == 0)

    # PiedG
    QRanges = m.segment(10).QRanges()
    assert(QRanges[0].min() == -pi / 2)
    assert(QRanges[0].max() == pi / 2)

def test_CoM():
    m = biorbd.Model('../../models/pyomecaman.bioMod')
    q_test_pyomecaman = [0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
    q = q_test_pyomecaman
    q_dot = [0]*m.nbQ()
    q_ddot = [0]*m.nbQ()
    for i in range(m.nbQ()):
        q_dot[i] = q_test_pyomecaman[i] * 10
        q_ddot[i] = q_test_pyomecaman[i] * 100

    if biorbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import Function, MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        q_dot_sym = MX.sym("q_dot", m.nbQdot(), 1)
        q_ddot_sym = MX.sym("q_ddot", m.nbQddot(), 1)

        CoM_func = Function(
            "Compute_CoM",
            [q_sym],
            [m.CoM(q_sym).to_mx()],
            ["q"],
            ["CoM"],
        ).expand()

        CoM_dot_func = Function(
            "Compute_CoM_dot",
            [q_sym, q_dot_sym],
            [m.CoMdot(q_sym, q_dot_sym).to_mx()],
            ["q", "q_dot"],
            ["CoM_dot"],
        ).expand()

        CoM_ddot_func = Function(
            "Compute_CoM_ddot",
            [q_sym, q_dot_sym, q_ddot_sym],
            [m.CoMddot(q_sym, q_dot_sym, q_ddot_sym).to_mx()],
            ["q", "q_dot", "q_ddot"],
            ["CoM_ddot"],
        ).expand()

        CoM = CoM_func(q)
        CoM_dot =  CoM_dot_func(q, q_dot)
        CoM_ddot = CoM_ddot_func(q, q_dot, q_ddot)

    elif not biorbd.currentLinearAlgebraBackend():
        # If Eigen backend is used
        CoM = m.CoM(q)
        CoM_dot = m.CoMdot(q, q_dot)
        CoM_ddot = m.CoMddot(q, q_dot, q_ddot)

    return CoM, CoM_dot, CoM_ddot


