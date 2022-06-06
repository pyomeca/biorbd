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
def test_get_range_q(brbd):
    biorbd_model = brbd.Model("../../models/pyomecaman.bioMod")

    range_min, range_max = brbd.get_range_q(biorbd_model)

    np.testing.assert_almost_equal(
        (range_min, range_max),
        (
            np.array(
                [
                    -10.0,
                    -10.0,
                    -3.14159265,
                    -3.14159265,
                    0.0,
                    -3.14159265,
                    0.0,
                    -0.26179939,
                    -2.0943951,
                    -1.57079633,
                    -0.26179939,
                    -2.0943951,
                    -1.57079633,
                ]
            ),
            np.array(
                [
                    10.0,
                    10.0,
                    3.14159265,
                    3.14159265,
                    3.14159265,
                    3.14159265,
                    3.14159265,
                    2.61799388,
                    0.0,
                    1.57079633,
                    2.61799388,
                    0.0,
                    1.57079633,
                ]
            ),
        ),
    )
