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


# --- Options --- #
@pytest.mark.parametrize("brbd", brbd_to_test)
def test_np_mx_to_generalized(brbd):
    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")

    q = brbd.GeneralizedCoordinates(biorbd_model)
    qdot = brbd.GeneralizedVelocity((biorbd_model.nbQdot()))
    qddot = brbd.GeneralizedAcceleration((biorbd_model.nbQddot()))
    tau = biorbd_model.InverseDynamics(q, qdot, qddot)
    biorbd_model.ForwardDynamics(q, qdot, tau)

    if brbd.currentLinearAlgebraBackend() == 1:
        tau = biorbd_model.InverseDynamics(q.to_mx(), qdot.to_mx(), qddot.to_mx())
        biorbd_model.ForwardDynamics(q, qdot, tau.to_mx())
    else:
        tau = biorbd_model.InverseDynamics(q.to_array(), qdot.to_array(), qddot.to_array())
        biorbd_model.ForwardDynamics(q, qdot, tau.to_array())


# --- Options --- #
@pytest.mark.parametrize("brbd", brbd_to_test)
def test_imu_to_array(brbd):
    m = brbd.Model("../../models/IMUandCustomRT/pyomecaman_withIMUs.bioMod")
    q = np.zeros((m.nbQ(),))

    if brbd.currentLinearAlgebraBackend() == 1:
        from casadi import MX

        q_sym = MX.sym("q", m.nbQ(), 1)
        imu_func = brbd.to_casadi_func("imu", m.IMU, q_sym)
        imu = imu_func(q)[:, :4]

    else:
        imu = m.IMU(q)[0].to_array()

    np.testing.assert_almost_equal(
        imu,
        np.array(
            [
                [0.99003329, -0.09933467, 0.09983342, 0.26719],
                [0.10925158, 0.98903828, -0.09933467, 0.04783],
                [-0.08887169, 0.10925158, 0.99003329, -0.20946],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_vector3d(brbd):
    biorbd_model = brbd.Model()
    vec = np.random.rand(
        3,
    )
    biorbd_model.setGravity(vec)

    if brbd.currentLinearAlgebraBackend() == 1:
        from casadi import MX

        vec = MX.ones(3, 1)
        biorbd_model.setGravity(vec)
