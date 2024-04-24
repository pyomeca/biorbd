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
def test_external_forces(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")

    # Remove the dampings in this test
    if brbd.currentLinearAlgebraBackend() == 1:
        jointDampings = [brbd.Scalar(0), brbd.Scalar(0), brbd.Scalar(0)]
    else:
        jointDampings = [0, 0, 0]
    m.segment(0).setJointDampings(jointDampings)

    force_set = m.externalForceSet()
    force_set.add(
        "PiedD",
        np.array([(0 + 1) * 11.1, (0 + 1) * 22.2, (0 + 1) * 33.3, (0 + 1) * 44.4, (0 + 1) * 55.5, (0 + 1) * 66.6]),
    )
    force_set.add(
        "PiedG",
        np.array([(1 + 1) * 11.1, (1 + 1) * 22.2, (1 + 1) * 33.3, (1 + 1) * 44.4, (1 + 1) * 55.5, (1 + 1) * 66.6]),
    )

    # Set to random values to test
    q = np.array([i * 1.1 for i in range(m.nbQ())])
    qdot = np.array([i * 1.1 for i in range(m.nbQ())])
    tau = np.array([i * 1.1 for i in range(m.nbQ())])

    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        qdot_sym = MX.sym("qdot", m.nbQdot(), 1)
        tau_sym = MX.sym("tau", m.nbGeneralizedTorque(), 1)

        brbd.to_casadi_func("ForwardDynamics", m.ForwardDynamics, q_sym, qdot_sym, tau_sym, force_set)
        # Call it twice because there is a chance they interact with each other
        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", m.ForwardDynamics, q_sym, qdot_sym, tau_sym, force_set)

        qddot = forward_dynamics(q, qdot, tau)
        qddot = np.array(qddot)[:, 0]

    elif brbd.currentLinearAlgebraBackend() == 0:
        # if Eigen backend is used
        m.ForwardDynamics(q, qdot, tau, force_set).to_array()
        # Call it twice because there is a chance they interact with each other
        qddot = m.ForwardDynamics(q, qdot, tau, force_set).to_array()

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
            3948.4748602722384
        ]
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_external_forces_with_point_of_application(brbd):
    m = brbd.Model("../../models/pyomecaman.bioMod")

    # Remove the dampings in this test
    if brbd.currentLinearAlgebraBackend() == 1:
        jointDampings = [brbd.Scalar(0), brbd.Scalar(0), brbd.Scalar(0)]
    else:
        jointDampings = [0, 0, 0]
    m.segment(0).setJointDampings(jointDampings)
    
    force_set = m.externalForceSet()
    force_set.add(
        "PiedD",
        np.array([(0 + 1) * 11.1, (0 + 1) * 22.2, (0 + 1) * 33.3, (0 + 1) * 44.4, (0 + 1) * 55.5, (0 + 1) * 66.6]),
        np.array([1, 2, 3]),
    )
    force_set.add(
        "PiedG",
        np.array([(1 + 1) * 11.1, (1 + 1) * 22.2, (1 + 1) * 33.3, (1 + 1) * 44.4, (1 + 1) * 55.5, (1 + 1) * 66.6]),
        np.array([1, 2, 3]),
    )

    # Set to random values to test
    q = np.array([i * 1.1 for i in range(m.nbQ())])
    qdot = np.array([i * 1.1 for i in range(m.nbQ())])
    tau = np.array([i * 1.1 for i in range(m.nbQ())])

    if brbd.currentLinearAlgebraBackend() == 1:
        # If CasADi backend is used
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        qdot_sym = MX.sym("qdot", m.nbQdot(), 1)
        tau_sym = MX.sym("tau", m.nbGeneralizedTorque(), 1)

        brbd.to_casadi_func("ForwardDynamics", m.ForwardDynamics, q_sym, qdot_sym, tau_sym, force_set)
        # Call it twice because there is a chance they interact with each other
        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", m.ForwardDynamics, q_sym, qdot_sym, tau_sym, force_set)

        qddot = forward_dynamics(q, qdot, tau)
        qddot = np.array(qddot)[:, 0]

    elif brbd.currentLinearAlgebraBackend() == 0:
        # if Eigen backend is used
        m.ForwardDynamics(q, qdot, tau, force_set).to_array()
        # Call it twice because there is a chance they interact with each other
        qddot = m.ForwardDynamics(q, qdot, tau, force_set).to_array()

    else:
        raise NotImplementedError("Backend not implemented in test")

    qddot_expected = np.array(
        [
            1.722118222873549,
            -10.504975014829379,
            -11.299572517471447,
            16.44657114839059,
            4.128030796214387,
            14.570146544931955,
            47.657007552920966,
            -207.44918614218255,
            -295.24781834578135,
            12442.979449875662,
            -87.20882953398387,
            429.1618388302072,
            -7972.557155115454,
        ],
    )
    np.testing.assert_almost_equal(qddot, qddot_expected)
