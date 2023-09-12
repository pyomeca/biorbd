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


def test_external_forces():
    
    external_forces = biorbd.ExternalForces(
        force_in_global=np.array([1, 2, 3]),
        torque_in_global=np.array([4, 5, 6]),
        application_point_in_global=np.array([0.1, 0.2, 0.3]),
    )
    np.testing.assert_array_equal(
        external_forces.spatial_vector.to_array(),
        np.array([4, 5, 6, 1, 2, 3]),
    )
    np.testing.assert_array_equal(
        external_forces._force_in_global,
        np.array([1, 2, 3]),
    )


