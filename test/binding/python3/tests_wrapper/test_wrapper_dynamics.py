import numpy as np
import pytest

from wrapper_tests_utils import evaluate, brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_dynamics(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert model.nb_q == model.internal.nbQ()
    assert model.nb_qdot == model.internal.nbQdot()
    assert model.nb_qddot == model.internal.nbQddot()
    assert model.nb_tau == model.internal.nbGeneralizedTorque()
    assert model.dof_names == [dof.to_string() for dof in model.internal.nameDof()]

    q = np.arange(model.nb_q)
    qdot = np.arange(model.nb_qdot)
    qddot = np.arange(model.nb_qdot)
    tau = np.arange(model.nb_tau)

    np.testing.assert_almost_equal(
        evaluate(brbd, model.forward_dynamics, q=q, qdot=qdot, tau=tau, ignore_contacts=True),
        [
            5.00389894,
            -16.15292551,
            -24.67896538,
            2.82389307,
            -2.85335065,
            815.30020415,
            115.55406325,
            100.4214427,
            -136.06994688,
            2240.33773397,
            -260.67363249,
            448.93542995,
            1385.1708001,
        ],
    )
    np.testing.assert_almost_equal(
        evaluate(brbd, model.inverse_dynamics, q=q, qdot=qdot, qddot=qddot),
        [
            7.94962580e02,
            5.18076639e02,
            2.85421296e-01,
            2.75129173e00,
            5.05735287e00,
            -1.91440054e00,
            3.46650123e00,
            -4.79279330e01,
            2.62285696e01,
            -4.49880204e00,
            1.34722842e02,
            -6.19256156e01,
            4.67175456e00,
        ],
        decimal=6,
    )


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_wrapper_dynamics(brbd)
