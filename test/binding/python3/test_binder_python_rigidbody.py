"""
Test for file IO
"""
import pytest
import numpy as np

brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except:
    pass
try:
    import biorbd_casadi

    brbd_to_test.append(biorbd_casadi)
except:
    pass


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_load_model(brbd):
    brbd.Model("../../models/pyomecaman.bioMod")


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_dof_ranges(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")
    pi = 3.14159265358979323846

    # Pelvis
    q_ranges = m.segment(0).QRanges()
    assert q_ranges[0].min() == -10
    assert q_ranges[0].max() == 10
    assert q_ranges[1].min() == -10
    assert q_ranges[1].max() == 10
    assert q_ranges[2].min() == -pi
    assert q_ranges[2].max() == pi

    # BrasD
    q_ranges = m.segment(3).QRanges()
    assert q_ranges[0].min() == -pi
    assert q_ranges[0].max() == pi
    assert q_ranges[1].min() == 0
    assert q_ranges[1].max() == pi

    # BrasG
    q_ranges = m.segment(4).QRanges()
    assert q_ranges[0].min() == -pi
    assert q_ranges[0].max() == pi
    assert q_ranges[1].min() == 0
    assert q_ranges[1].max() == pi

    # CuisseD
    q_ranges = m.segment(5).QRanges()
    assert q_ranges[0].min() == -pi / 12
    assert q_ranges[0].max() == pi / 2 + pi / 3

    # JambeD
    q_ranges = m.segment(6).QRanges()
    assert q_ranges[0].min() == -pi / 2 - pi / 6
    assert q_ranges[0].max() == 0

    # PiedD
    q_ranges = m.segment(7).QRanges()
    assert q_ranges[0].min() == -pi / 2
    assert q_ranges[0].max() == pi / 2

    # CuisseG
    q_ranges = m.segment(8).QRanges()
    assert q_ranges[0].min() == -pi / 12
    assert q_ranges[0].max() == pi / 2 + pi / 3

    # JambeG
    q_ranges = m.segment(9).QRanges()
    assert q_ranges[0].min() == -pi / 2 - pi / 6
    assert q_ranges[0].max() == 0

    # PiedG
    q_ranges = m.segment(10).QRanges()
    assert q_ranges[0].min() == -pi / 2
    assert q_ranges[0].max() == pi / 2


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_forward_dynamics(brbd):
    m = brbd.Model("../../models/pyomecaman_withActuators.bioMod")

    # Remove the dampings in this test
    if brbd.currentLinearAlgebraBackend() == 1:
        jointDampings = [brbd.Scalar(0), brbd.Scalar(0), brbd.Scalar(0)]
    else:
        jointDampings = [0, 0, 0]
    m.segment(0).setJointDampings(jointDampings)

    q = np.array([i * 1.1 for i in range(m.nbQ())])
    qdot = np.array([i * 1.1 for i in range(m.nbQ())])
    tau = np.array([i * 1.1 for i in range(m.nbQ())])

    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        qdot_sym = MX.sym("qdot", m.nbQdot(), 1)
        tau_sym = MX.sym("tau", m.nbGeneralizedTorque(), 1)
        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", m.ForwardDynamics, q_sym, qdot_sym, tau_sym)

        qddot = forward_dynamics(q, qdot, tau)
        qddot = np.array(qddot)[:, 0]

    elif brbd.currentLinearAlgebraBackend() == 0:
        # if Eigen backend is used
        qddot = m.ForwardDynamics(q, qdot, tau).to_array()
    else:
        raise NotImplementedError("Backend not implemented in test")

    qddot_expected = np.array(
        [
            20.554883896960259,
            -22.317642013324736,
            -77.406439058256126,
            17.382961188212313,
            -63.426361095191858,
            93.816468824985876,
            106.46105024484631,
            95.116641811710167,
            -268.1961283528546,
            2680.3632159799949,
            -183.4582596257801,
            755.89411812405604,
            163.60239754283589,
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_forward_dynamics_with_external_forces(brbd):
    m = brbd.Model("../../models/pyomecaman_withActuators.bioMod")

    # Remove the dampings in this test
    if brbd.currentLinearAlgebraBackend() == 1:
        jointDampings = [brbd.Scalar(0), brbd.Scalar(0), brbd.Scalar(0)]
    else:
        jointDampings = [0, 0, 0]
    m.segment(0).setJointDampings(jointDampings)
    
    q = np.array([i * 1.1 for i in range(m.nbQ())])
    qdot = np.array([i * 1.1 for i in range(m.nbQ())])
    tau = np.array([i * 1.1 for i in range(m.nbQ())])

    # With external forces
    external_forces = m.externalForceSet()
    external_forces.add("PiedD", np.array((11.1, 22.2, 33.3, 44.4, 55.5, 66.6)))
    external_forces.add("PiedG", np.array((11.1 * 2, 22.2 * 2, 33.3 * 2, 44.4 * 2, 55.5 * 2, 66.6 * 2)))

    if brbd.currentLinearAlgebraBackend() == 1:
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        qdot_sym = MX.sym("qdot", m.nbQdot(), 1)
        tau_sym = MX.sym("tau", m.nbGeneralizedTorque(), 1)
        forward_dynamics = brbd.to_casadi_func(
            "ForwardDynamics", m.ForwardDynamics, q_sym, qdot_sym, tau_sym, external_forces
        )

        qddot = forward_dynamics(q, qdot, tau)
        qddot = np.array(qddot)[:, 0]

    elif brbd.currentLinearAlgebraBackend() == 0:
        # if Eigen backend is used
        qddot = m.ForwardDynamics(q, qdot, tau, external_forces).to_array()
    else:
        raise NotImplementedError("Backend not implemented in test")

    qddot_expected = np.array(
        [
            8.8871711208009998,
            -13.647827029817943,
            -33.606145294752132,
            16.922669487341341,
            -21.882821189868423,
            41.15364990805439,
            68.892537246574463,
            -324.59756885799197,
            -447.99217990207387,
            18884.241415786601,
            -331.24622725851572,
            1364.7620674666462,
            3948.4748602722384,
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_com(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")

    q = np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    q_dot = np.array([1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])
    q_ddot = np.array([10, 10, 10, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30])

    expected_com = np.array([-0.0034679564024098523, 0.15680579877453169, 0.07808112642459612])
    expected_com_dot = np.array([-0.05018973433722229, 1.4166208451420528, 1.4301750486035787])
    expected_com_ddot = np.array([-0.7606169667295027, 11.508107073695976, 16.58853835505851])

    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        q_dot_sym = MX.sym("q_dot", m.nbQdot(), 1)
        q_ddot_sym = MX.sym("q_ddot", m.nbQddot(), 1)

        com_func = brbd.to_casadi_func("Compute_CoM", m.CoM, q_sym)
        com_dot_func = brbd.to_casadi_func("Compute_CoM_dot", m.CoMdot, q_sym, q_dot_sym)
        com_ddot_func = brbd.to_casadi_func("Compute_CoM_ddot", m.CoMddot, q_sym, q_dot_sym, q_ddot_sym)

        com = np.array(com_func(q))
        com_dot = np.array(com_dot_func(q, q_dot))
        com_ddot = np.array(com_ddot_func(q, q_dot, q_ddot))

    elif not brbd.currentLinearAlgebraBackend():
        # If Eigen backend is used
        com = m.CoM(q).to_array()
        com_dot = m.CoMdot(q, q_dot).to_array()
        com_ddot = m.CoMddot(q, q_dot, q_ddot).to_array()
    else:
        raise NotImplementedError("Backend not implemented in test")

    np.testing.assert_almost_equal(com.squeeze(), expected_com)
    np.testing.assert_almost_equal(com_dot.squeeze(), expected_com_dot)
    np.testing.assert_almost_equal(com_ddot.squeeze(), expected_com_ddot)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_set_vector3d(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")
    m.setGravity(np.array((0, 0, -2)))
    if brbd.currentLinearAlgebraBackend() == 1:
        from casadi import MX

        get_gravity = brbd.to_casadi_func("Compute_Markers", m.getGravity)()["o0"]
    else:
        get_gravity = m.getGravity().to_array()
    assert get_gravity[2] == -2


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_set_scalar(brbd):
    def check_value(target):
        if brbd.currentLinearAlgebraBackend() == 1:
            assert m.segment(0).characteristics().mass().to_mx() == target
        else:
            assert m.segment(0).characteristics().mass() == target

    m = brbd.Model("../../models/pyomecaman.bioMod")

    m.segment(0).characteristics().setMass(10)
    check_value(10)

    m.segment(0).characteristics().setMass(11.0)
    check_value(11.0)

    with pytest.raises(ValueError, match="Scalar must be a 1x1 array or a float"):
        m.segment(0).characteristics().setMass(np.array([]))

    m.segment(0).characteristics().setMass(np.array((12,)))
    check_value(12.0)

    m.segment(0).characteristics().setMass(np.array([[13]]))
    check_value(13.0)

    with pytest.raises(ValueError, match="Scalar must be a 1x1 array or a float"):
        m.segment(0).characteristics().setMass(np.array([[[14]]]))

    if brbd.currentLinearAlgebraBackend() == 1:
        from casadi import MX

        m.segment(0).characteristics().setMass(MX(15))
        check_value(15.0)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_markers(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")

    q = np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    q_dot = np.array([1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])

    expected_markers_last = np.array([-0.11369, 0.63240501, -0.56253268])
    expected_markers_last_dot = np.array([0.0, 4.16996219, 3.99459262])

    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        q_dot_sym = MX.sym("q_dot", m.nbQdot(), 1)

        markers_func = brbd.to_casadi_func("Compute_Markers", m.markers, q_sym)
        markers_velocity_func = brbd.to_casadi_func("Compute_MarkersVelocity", m.markersVelocity, q_sym, q_dot_sym)

        markers = np.array(markers_func(q))
        markers_dot = np.array(markers_velocity_func(q, q_dot))

    elif not brbd.currentLinearAlgebraBackend():
        # If Eigen backend is used
        markers = np.array([mark.to_array() for mark in m.markers(q)]).T
        markers_dot = np.array([mark.to_array() for mark in m.markersVelocity(q, q_dot)]).T

    else:
        raise NotImplementedError("Backend not implemented in test")

    np.testing.assert_almost_equal(markers[:, -1], expected_markers_last)
    np.testing.assert_almost_equal(markers_dot[:, -1], expected_markers_last_dot)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_forward_dynamics_constraints_direct(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")

    # Remove the dampings in this test
    if brbd.currentLinearAlgebraBackend() == 1:
        jointDampings = [brbd.Scalar(0), brbd.Scalar(0), brbd.Scalar(0)]
    else:
        jointDampings = [0, 0, 0]
    m.segment(0).setJointDampings(jointDampings)
    
    q = np.array([1.0 for _ in range(m.nbQ())])
    qdot = np.array([1.0 for _ in range(m.nbQ())])
    tau = np.array([1.0 for _ in range(m.nbQ())])
    cs = m.getConstraints()

    qddot_expected = np.array(
        [
            1.9402069774422919,
            -9.1992692111538243,
            2.9930159570454702,
            5.2738378853554133,
            8.9387539396273699,
            6.0938738229550751,
            9.9560407885164217,
            38.6297746304162,
            -52.159023390563554,
            36.702385054876714,
            38.629774630416208,
            -52.159023390563561,
            36.70238505487675,
        ]
    )
    contact_forces_expected = np.array(
        [
            -16.344680827308579,
            -30.485214214095951,
            112.8234134576031,
            -16.344680827308611,
            -30.485214214095965,
            112.82341345760311,
        ]
    )

    np.testing.assert_almost_equal(cs.nbContacts(), contact_forces_expected.size)

    if brbd.currentLinearAlgebraBackend() == 1:
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

        qddot, cs_forces = dyn_func(q, qdot)
        qddot = np.array(qddot)
        cs_forces = np.array(cs_forces)

    elif brbd.currentLinearAlgebraBackend() == 0:
        # if Eigen backend is used
        qddot = m.ForwardDynamicsConstraintsDirect(q, qdot, tau, cs).to_array()
        cs_forces = cs.getForce().to_array()

    else:
        raise NotImplementedError("Backend not implemented in test")

    np.testing.assert_almost_equal(qddot.squeeze(), qddot_expected)
    np.testing.assert_almost_equal(cs_forces.squeeze(), contact_forces_expected)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_name_to_index(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")

    # Index of a segment
    np.testing.assert_equal(brbd.segment_index(m, "Pelvis"), 0)
    np.testing.assert_equal(brbd.segment_index(m, "PiedG"), 10)
    with pytest.raises(ValueError, match="dummy is not in the biorbd model"):
        brbd.segment_index(m, "dummy")

    # Index of a marker
    np.testing.assert_equal(brbd.marker_index(m, "pelv1"), 0)
    np.testing.assert_equal(brbd.marker_index(m, "piedg6"), 96)
    with pytest.raises(ValueError, match="dummy is not in the biorbd model"):
        brbd.marker_index(m, "dummy")
