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
    
def test_forward_dynamics():
    m = biorbd.Model("../../models/pyomecaman_withActuators.bioMod")

    q = np.array([i*1.1 for i in range(m.nbQ())])
    qdot = np.array([i*1.1 for i in range(m.nbQ())])
    tau = np.array([i*1.1 for i in range(m.nbQ())])
    
    qddot = m.ForwardDynamics(q, qdot, tau).to_array()
    qddot_expected = np.array(
        [
            20.554883896960259, -22.317642013324736, -77.406439058256126, 
            17.382961188212313, -63.426361095191858, 93.816468824985876, 
            106.46105024484631, 95.116641811710167, -268.1961283528546, 
            2680.3632159799949, -183.4582596257801, 755.89411812405604,
            163.60239754283589
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)
    
    # With external forces
    f_ext = biorbd.VecBiorbdSpatialVector(
        [
            biorbd.SpatialVector(11.1, 22.2, 33.3, 44.4, 55.5, 66.6), 
            biorbd.SpatialVector(11.1*2, 22.2*2, 33.3*2, 44.4*2, 55.5*2, 66.6*2)
        ]
    )

    qddot = m.ForwardDynamics(q, qdot, tau, f_ext).to_array()
    qddot_expected = np.array(
        [
            8.8871711208009998, -13.647827029817943, -33.606145294752132, 
            16.922669487341341, -21.882821189868423, 41.15364990805439, 
            68.892537246574463, -324.59756885799197, -447.99217990207387, 
            18884.241415786601, -331.24622725851572, 1364.7620674666462,
            3948.4748602722384
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)

def test_CoM():
    m = biorbd.Model('../../models/pyomecaman.bioMod')

    q = np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    q_dot = np.array([1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])
    q_ddot = np.array([10, 10, 10, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30])

    expected_CoM = np.array([-0.0034679564024098523, 0.15680579877453169, 0.07808112642459612])
    expected_CoM_dot = np.array([-0.05018973433722229, 1.4166208451420528, 1.4301750486035787])
    expected_CoM_ddot = np.array([-0.7606169667295027, 11.508107073695976, 16.58853835505851])

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

        CoM = np.array(CoM_func(q))
        CoM_dot = np.array(CoM_dot_func(q, q_dot))
        CoM_ddot = np.array(CoM_ddot_func(q, q_dot, q_ddot))

    elif not biorbd.currentLinearAlgebraBackend():
        # If Eigen backend is used
        CoM = m.CoM(q).to_array()
        CoM_dot = m.CoMdot(q, q_dot).to_array()
        CoM_ddot = m.CoMddot(q, q_dot, q_ddot).to_array()

    np.testing.assert_almost_equal(CoM.squeeze(), expected_CoM)
    np.testing.assert_almost_equal(CoM_dot.squeeze(), expected_CoM_dot)
    np.testing.assert_almost_equal(CoM_ddot.squeeze(), expected_CoM_ddot)

def test_forward_dynamics_constraints_direct():
    m = biorbd.Model("../../models/pyomecaman.bioMod")

    q = np.array([1.0 for i in range(m.nbQ())])
    qdot = np.array([1.0 for i in range(m.nbQ())])
    tau = np.array([1.0 for i in range(m.nbQ())])
    cs = m.getConstraints()

    qddot_expected = np.array(
        [
            1.9402069774422919, -9.1992692111538243, 2.9930159570454702,
            5.2738378853554133, 8.9387539396273699, 6.0938738229550751, 9.9560407885164217,
            38.6297746304162, -52.159023390563554, 36.702385054876714, 38.629774630416208, -52.159023390563561,
            36.70238505487675
        ]
    )
    contact_forces_expected = np.array(
        [
            -16.344680827308579, -30.485214214095951, 112.8234134576031, -16.344680827308611,
            -30.485214214095965, 112.82341345760311
        ]
    )

    np.testing.assert_almost_equal(cs.nbContacts(), contact_forces_expected.size)

    if biorbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import Function, MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        qdot_sym = MX.sym("qdot", m.nbQdot(), 1)

        dyn_func = Function(
            "Compute_qddot_with_dyn",
            [q_sym, qdot_sym],
            [m.ForwardDynamicsConstraintsDirect(q, qdot, tau, cs).to_mx(), cs.getForce().to_mx()],
            ["q", "qdot_sym"],
            ["qddot", "cs_forces"],
        ).expand()

        qddot, cs_forces = dyn_func(q,qdot)
        qddot = np.array(qddot)
        cs_forces = np.array(cs_forces)

    elif biorbd.currentLinearAlgebraBackend() == 0:
        # if Eigen backend is used
        qddot = m.ForwardDynamicsConstraintsDirect(q, qdot, tau, cs).to_array()
        cs_forces = cs.getForce().to_array()

    np.testing.assert_almost_equal(qddot.squeeze(), qddot_expected)
    np.testing.assert_almost_equal(cs_forces.squeeze(), contact_forces_expected)