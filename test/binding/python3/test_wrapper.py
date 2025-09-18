import re

brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except:
    pass
try:
    import biorbd_casadi
    from casadi import MX

    brbd_to_test.append(biorbd_casadi)
except:
    pass

import numpy as np
import pytest


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_segments(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert len(model.segments) == len(model.internal.segments())

    segment = model.segments[0]
    # Name
    assert segment.name == "Pelvis"

    # Mass
    assert segment.mass == 9.03529
    segment.mass = 100
    assert segment.mass == 100


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_dynamics(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert model.nb_q == model.internal.nbQ()
    assert model.nb_qdot == model.internal.nbQdot()
    assert model.nb_tau == model.internal.nbGeneralizedTorque()
    assert model.dof_names == [dof.to_string() for dof in model.internal.nameDof()]

    if brbd.currentLinearAlgebraBackend() == 1:
        q_sym = MX.sym("q", (model.nb_q,))
        qdot_sym = MX.sym("qdot", (model.nb_qdot,))
        qddot_sym = MX.sym("qddot", (model.nb_qdot,))
        tau_sym = MX.sym("tau", (model.nb_tau,))

        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", model.forward_dynamics, q_sym, qdot_sym, tau_sym)
        forward_dynamics_internal = brbd.to_casadi_func(
            "ForwardDynamicsInternal", model.internal.ForwardDynamics, q_sym, qdot_sym, tau_sym
        )

        inverse_dynamics = brbd.to_casadi_func("InverseDynamics", model.inverse_dynamics, q_sym, qdot_sym, qddot_sym)
        inverse_dynamics_internal = brbd.to_casadi_func(
            "InverseDynamicsInternal", model.internal.InverseDynamics, q_sym, qdot_sym, qddot_sym
        )

    else:
        forward_dynamics = model.forward_dynamics
        forward_dynamics_internal = model.internal.ForwardDynamics

        inverse_dynamics = model.inverse_dynamics
        inverse_dynamics_internal = model.internal.InverseDynamics

    q = np.arange(model.nb_q)
    qdot = np.arange(model.nb_qdot)
    qddot = np.arange(model.nb_qdot)
    tau = np.arange(model.nb_tau)

    np.testing.assert_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=True), forward_dynamics_internal(q, qdot, tau).to_array()
    )
    np.testing.assert_almost_equal(
        inverse_dynamics(q, qdot, qddot), inverse_dynamics_internal(q, qdot, qddot).to_array()
    )


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_external_forces(brbd):
    # Load a predefined model
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    # Segment on which the external force will be applied
    segment = model.segments[0]

    # Testing failing cases
    with pytest.raises(ValueError, match=re.escape("The input vector must be of size 3 (force) or 6 (spatial vector)")):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0], point_of_application=[0, 0, 0])
        model.external_force_set.add(segment_name=segment.name, force=[0, 0, 0, 0], point_of_application=[0, 0, 0])
    with pytest.raises(
        ValueError,
        match="The point of application must be provided when adding a force in the global reference frame",
    ):
        model.external_force_set.add(segment_name=segment.name, force=[0, 0, 9.81 * segment.mass])
    with pytest.raises(
        ValueError,
        match=re.escape(
            "Adding a force in local reference frame is not implemented (and probably not what you want). "
            "You probably want to add a spatial vector in the local reference frame instead or a force in the global reference frame."
        ),
    ):
        model.external_force_set.add(
            segment_name=segment.name,
            force=[0, 0, 9.81 * segment.mass],
            reference_frame=brbd.ReferenceFrame.LOCAL,
            point_of_application=[0, 0, 0],
        )
    with pytest.raises(
        ValueError,
        match="The point of application must be provided when adding a spatial vector in the local reference frame",
    ):
        model.external_force_set.add(
            segment_name=segment.name,
            force=[0, 0, 0, 0, 0, 9.81 * segment.mass],
            reference_frame=brbd.ReferenceFrame.LOCAL,
        )

    if brbd.currentLinearAlgebraBackend() == 1:
        q_sym = MX.sym("q", (model.nb_q,))
        qdot_sym = MX.sym("qdot", (model.nb_qdot,))
        tau_sym = MX.sym("tau", (model.nb_tau,))

        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", model.forward_dynamics, q_sym, qdot_sym, tau_sym)
    else:
        forward_dynamics = model.forward_dynamics

    # Computing forward dynamics as reference
    q = [0] * model.nb_q
    qdot = [0] * model.nb_qdot
    tau = [0] * model.nb_tau
    ref_qddot = forward_dynamics(q=q, qdot=qdot, tau=tau, ignore_contacts=True, ignore_external_forces=True)
    np.testing.assert_array_almost_equal(
        ref_qddot,
        [0, -9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    )

    # Test adding one external force to the model
    model.external_force_set.add(
        segment_name=segment.name, force=[0, 0, 9.81 * segment.mass], point_of_application=[0, 0, 0]
    )
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=False),
        [
            -0.195144,
            -0.141795,
            5.03609,
            15.715842,
            -8.472491,
            -15.715842,
            -8.472491,
            -18.12955,
            27.010416,
            -13.916956,
            -18.12955,
            27.010416,
            -13.916956,
        ],
    )
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=True),
        [
            0.697101,
            -7.923693,
            2.846458,
            2.374355,
            -3.754325,
            -2.374355,
            -3.754325,
            -7.568292,
            6.753389,
            -18.305875,
            -7.568292,
            6.753389,
            -18.305875,
        ],
    )

    # Add an extra external force to the model that pushes upward even more
    model.external_force_set.add(segment_name=segment.name, force=[0, 0, 1], point_of_application=[0, 0, 0])
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=False),
        [
            -0.185022,
            -0.139734,
            5.062955,
            15.709644,
            -8.503352,
            -15.709644,
            -8.503352,
            -18.190191,
            27.044657,
            -13.917421,
            -18.190191,
            27.044657,
            -13.917421,
        ],
    )
    np.testing.assert_array_almost_equal(
        forward_dynamics(q, qdot, tau, ignore_contacts=True),
        [
            0.704966,
            -7.902412,
            2.878572,
            2.401142,
            -3.796682,
            -2.401142,
            -3.796682,
            -7.653678,
            6.829581,
            -18.512403,
            -7.653678,
            6.829581,
            -18.512403,
        ],
    )

    # Remove all external forces
    model.external_force_set.reset()
    np.testing.assert_array_almost_equal(forward_dynamics(q, qdot, tau, ignore_contacts=True), ref_qddot)
