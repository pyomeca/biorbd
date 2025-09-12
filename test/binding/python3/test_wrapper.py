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
def test_wrapper_methods(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert model.nb_q == model.internal_model.nbQ()
    assert model.nb_qdot == model.internal_model.nbQdot()
    assert model.nb_tau == model.internal_model.nbGeneralizedTorque()
    assert model.dof_names == [dof.to_string() for dof in model.internal_model.nameDof()]

    if brbd.currentLinearAlgebraBackend() == 1:
        q_sym = MX.sym("q", (model.nb_q,))
        qdot_sym = MX.sym("qdot", (model.nb_qdot,))
        qddot_sym = MX.sym("qddot", (model.nb_qdot,))
        tau_sym = MX.sym("tau", (model.nb_tau,))

        forward_dynamics = brbd.to_casadi_func("ForwardDynamics", model.forward_dynamics, q_sym, qdot_sym, tau_sym)
        forward_dynamics_internal = brbd.to_casadi_func(
            "ForwardDynamicsInternal", model.internal_model.ForwardDynamics, q_sym, qdot_sym, tau_sym
        )

        inverse_dynamics = brbd.to_casadi_func("InverseDynamics", model.inverse_dynamics, q_sym, qdot_sym, qddot_sym)
        invers_dynamics_internal = brbd.to_casadi_func(
            "InverseDynamicsInternal", model.internal_model.InverseDynamics, q_sym, qdot_sym, qddot_sym
        )

    else:
        forward_dynamics = model.forward_dynamics
        forward_dynamics_internal = model.internal_model.ForwardDynamics

        inverse_dynamics = model.inverse_dynamics
        invers_dynamics_internal = model.internal_model.InverseDynamics

    q = np.arange((model.nb_q,))
    qdot = np.arange((model.nb_qdot,))
    qddot = np.arange((model.nb_qdot,))
    tau = np.arange((model.nb_tau,))

    assert forward_dynamics(q, qdot, tau) == forward_dynamics_internal(q, qdot, tau)
    assert inverse_dynamics(q, qdot, qddot) == invers_dynamics_internal(q, qdot, qddot)
