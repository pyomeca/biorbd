"""
Test for file IO
"""
from pathlib import Path

import numpy as np
import pytest

import biorbd


# --- Options --- #
def test_np_mx_to_generalized():
    biorbd_model = biorbd.Model("../../models/pyomecaman.bioMod")

    q = biorbd.GeneralizedCoordinates(biorbd_model)
    qdot = biorbd.GeneralizedVelocity((biorbd_model.nbQdot()))
    qddot = biorbd.GeneralizedAcceleration((biorbd_model.nbQddot()))
    tau = biorbd_model.InverseDynamics(q, qdot, qddot)
    biorbd_model.ForwardDynamics(q, qdot, tau)

    if biorbd.currentLinearAlgebraBackend() == 1:
        tau = biorbd_model.InverseDynamics(q.to_mx(), qdot.to_mx(), qddot.to_mx())
        biorbd_model.ForwardDynamics(q, qdot, tau.to_mx())
    else:
        tau = biorbd_model.InverseDynamics(q.to_array(), qdot.to_array(), qddot.to_array())
        biorbd_model.ForwardDynamics(q, qdot, tau.to_array())
