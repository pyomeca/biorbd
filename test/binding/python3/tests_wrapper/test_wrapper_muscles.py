try:
    from casadi import MX, Function
except ModuleNotFoundError as e:
    pass

import numpy as np
import pytest

from wrapper_tests_utils import evaluate, brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_muscles(brbd):
    # Load a predefined model
    model = brbd.Biorbd("../../models/arm26.bioMod")
    muscles = model.muscles
    nmus = len(muscles)

    assert nmus == len(model.internal.muscles())

    # Choose a state (position/velocity) to compute dynamics from
    activations = [0.5] * nmus
    q = [0.1] * model.nb_q
    qdot = [0.1] * model.nb_qdot

    # 1. Using the "low-level" way
    muscles.activations = activations
    muscles.update_geometry(q=q, qdot=qdot)
    tau_ref = [-8.99327207, -11.62806138]
    if brbd.backend == brbd.CASADI:
        q_sym = MX.sym("q", model.nb_q, 1)
        qdot_sym = MX.sym("qdot", model.nb_qdot, 1)
        muscles.update_geometry(q=q_sym, qdot=qdot_sym)
        joint_torque = Function("joint_torque", [q_sym, qdot_sym], [muscles.joint_torque()])
        np.testing.assert_almost_equal(np.array(joint_torque(q, qdot))[:, 0], tau_ref)
    else:
        np.testing.assert_almost_equal(muscles.joint_torque(), tau_ref)
    np.testing.assert_almost_equal(evaluate(brbd, muscles.joint_torque, q=q, qdot=qdot), tau_ref)

    # 2. The "high-level" way, where the kinematics is updated internally (using a different q so we are not accidentally up to date)
    tau = evaluate(brbd, muscles.joint_torque, activations=activations, q=np.array(q) * 2, qdot=qdot)
    np.testing.assert_almost_equal(tau, [-10.11532606, -10.13974674])

    # Validate forces and length jacobian
    jacobian = np.array(evaluate(brbd, muscles.lengths_jacobian, q=np.array(q) * 2)).squeeze()
    forces = evaluate(brbd, muscles.forces, q=np.array(q) * 2, qdot=qdot)
    np.testing.assert_almost_equal(-jacobian.T @ forces, [-10.11532606, -10.13974674])

    # Test the muscle forces for internal and explicit kinematics update
    forces_ref = [403.47878369, 341.14054494, 214.28139395, 239.38801482, 201.51684182, 493.98451373]
    if brbd.backend == brbd.CASADI:
        q_sym = MX.sym("q", model.nb_q, 1)
        qdot_sym = MX.sym("qdot", model.nb_qdot, 1)
        muscles.update_geometry(q=q_sym, qdot=qdot_sym)
        forces = Function("forces", [q_sym, qdot_sym], [muscles.forces()])
        np.testing.assert_almost_equal(np.array(forces(np.array(q) * 2, qdot))[:, 0], forces_ref)
    else:
        np.testing.assert_almost_equal(muscles.forces(), forces_ref)
    np.testing.assert_almost_equal(evaluate(brbd, muscles.forces, q=np.array(q) * 2, qdot=qdot), forces_ref)

    forces_ref = [245.63266637, 225.70738926, 132.04335764, 143.63280889, 117.39623324, 296.81063517]
    if brbd.backend == brbd.CASADI:
        q_sym = MX.sym("q", model.nb_q, 1)
        qdot_sym = MX.sym("qdot", model.nb_qdot, 1)
        muscles.update_geometry(q=q_sym, qdot=qdot_sym)
        joint_torque = Function("joint_torque", [q_sym, qdot_sym], [muscles.forces(activations=[0.3] * nmus)])
        np.testing.assert_almost_equal(np.array(joint_torque(np.array(q) * 2, qdot))[:, 0], forces_ref)
    else:
        np.testing.assert_almost_equal(muscles.forces(activations=[0.3] * nmus), forces_ref)
    np.testing.assert_almost_equal(
        evaluate(brbd, muscles.forces, activations=[0.3] * nmus, q=np.array(q) * 2, qdot=qdot), forces_ref
    )

    # Test activation dot
    initial_activations = [0.5] * nmus
    excitations = np.arange(1, nmus + 1) / nmus  # Varying excitations from 0.1 to 1.0
    activations_dot = evaluate(brbd, muscles.activations_dot, excitations=excitations, activations=initial_activations)
    np.testing.assert_almost_equal(
        activations_dot,
        [-10.416666666666668, -5.208333333333334, 0.0, 13.33333333333333, 26.666666666666668, 40.0],
    )

    # Get an change the properties of a specific muscle
    muscle = muscles[0]
    assert muscle.name == "TRIlong"

    assert muscle.optimal_length == 0.134
    muscle.optimal_length = 0.2
    assert muscle.optimal_length == 0.2

    assert muscle.maximal_isometric_force == 798.52
    muscle.maximal_isometric_force = 1000.0
    assert muscle.maximal_isometric_force == 1000.0

    assert muscle.pcsa == 0.0
    muscle.pcsa = 0.02
    assert muscle.pcsa == 0.02

    assert muscle.tendon_slack_length == 0.143
    muscle.tendon_slack_length = 0.3
    assert muscle.tendon_slack_length == 0.3

    assert muscle.pennation_angle == 0.20943951
    muscle.pennation_angle = 0.1
    assert muscle.pennation_angle == 0.1

    assert muscle.maximal_contraction_velocity == 10.0
    muscle.maximal_contraction_velocity = 15.0
    assert muscle.maximal_contraction_velocity == 15.0

    if brbd.backend == brbd.CASADI:
        q_sym = MX.sym("q", model.nb_q, 1)
        qdot_sym = MX.sym("qdot", model.nb_qdot, 1)
        muscles.update_geometry(q=q_sym, qdot=qdot_sym)
        force = Function("force", [q_sym, qdot_sym], [muscle.force])
        forces = Function("forces", [q_sym, qdot_sym], [muscles.forces()])
        np.testing.assert_almost_equal(force(q, qdot), forces(q, qdot)[0])
    else:
        # The non-updated value is not suppose to reflect the changes properly
        assert muscle.force == muscles.forces()[0]
        np.testing.assert_almost_equal(
            muscles.forces(), [441.00852545, 341.14054494, 214.28139395, 239.38801482, 201.51684182, 493.98451373]
        )
        muscles.update_geometry(q=q, qdot=qdot)
        assert muscle.force == muscles.forces()[0]

    np.testing.assert_almost_equal(
        evaluate(brbd, muscles.forces, q=q, qdot=qdot),
        [37.43286925, 349.67505477, 212.67611275, 231.48743683, 194.03986176, 494.72543227],
    )


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_static_optimization(brbd):
    if brbd.backend == brbd.CASADI:
        assert brbd.has_static_optimization is False
        return
    assert brbd.has_static_optimization is True

    # Load a predefined model
    model = brbd.Biorbd("../../models/arm26.bioMod")
    n_frames = 3

    q = []
    qdot = []
    qddot = []
    tau = []
    for i in range(n_frames):
        # The q would typically come from an inverse kinematics analysis
        q.append([0] * model.nb_q)
        # The qdot and qddot would typically come from a numerical differentiation of the kinematics
        qdot.append([0] * model.nb_qdot)
        qddot.append([0] * model.nb_qddot)

        # The tau would typically come from an inverse dynamics analysis as follow
        tau.append(model.inverse_dynamics(q[i], qdot[i], qddot[i]))

    # Proceed with the static optimization. When perform is called, all the frames are processed at once, even though
    # it is a loop. That is so the initial guess is dependent of the previous frame. So the first "frame" of the loop is
    # very long (as it computes everythin). Then, the following frames are very fast (as it only returns the precomputed
    # results)
    optim = brbd.StaticOptimization(model)
    muscle_activations = []
    for value in optim.perform_frames(q, qdot, tau):
        muscle_activations.append(value)
    assert len(muscle_activations) == n_frames
    np.testing.assert_almost_equal(
        muscle_activations[0], [0.00012686, 0.00012018, 0.00089968, 0.00010025, 0.00010027, 0.00010081], decimal=5
    )
    np.testing.assert_almost_equal(
        muscle_activations[-1], [0.00012686, 0.00012018, 0.00089968, 0.00010025, 0.00010027, 0.00010081], decimal=5
    )


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_muscles(brbd)
        test_static_optimization(brbd)
