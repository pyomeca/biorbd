"""
Test for file IO
"""

import pytest
import numpy as np

brbd_to_test = []
try:
    import biorbd

    brbd_to_test.append(biorbd)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd: {e}")
    pass

try:
    import biorbd_casadi

    brbd_to_test.append(biorbd_casadi)
except ModuleNotFoundError as e:
    print(f"Error importing biorbd_casadi: {e}")
    pass

if not brbd_to_test:
    raise RuntimeError("No biorbd version could be imported")


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


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_multiply(brbd):
    if brbd == biorbd_casadi:
        return

    np.random.seed(42)
    b1 = np.random.rand(3, 3)
    a1 = np.random.rand(3, 3)
    a = brbd.Matrix3d(a1[0, 0], a1[0, 1], a1[0, 2], a1[1, 0], a1[1, 1], a1[1, 2], a1[2, 0], a1[2, 1], a1[2, 2])
    b = brbd.Matrix3d(b1[0, 0], b1[0, 1], b1[0, 2], b1[1, 0], b1[1, 1], b1[1, 2], b1[2, 0], b1[2, 1], b1[2, 2])
    c = a.multiply(b)
    np.testing.assert_almost_equal(
        c.to_array(),
        np.array(
            [[0.3338605, 1.5164991, 1.1045433], [0.4494628, 0.9820364, 0.7517644], [0.2813093, 0.6763643, 0.4971501]]
        ),
    )

    a = brbd.Rotation.fromEulerAngles(np.array([1, 2, 3]), "xyz")
    b = brbd.Rotation.fromEulerAngles(np.array([2, 3, 4]), "xyz")
    c = a.multiply(b)
    np.testing.assert_almost_equal(
        c.to_array(),
        np.array(
            [
                [-0.38048103, -0.78702507, 0.48561892],
                [-0.84379757, 0.08054074, -0.5305835],
                [0.37847041, -0.61164102, -0.69473406],
            ]
        ),
    )

    a = brbd.Rotation.fromEulerAngles(np.array([1, 2, 3]), "xyz")
    b = brbd.Rotation.fromEulerAngles(np.array([2, 3, 4]), "xyz")
    c = a.multiply(b)
    np.testing.assert_almost_equal(
        c.to_array(),
        np.array(
            [
                [-0.38048103, -0.78702507, 0.48561892],
                [-0.84379757, 0.08054074, -0.5305835],
                [0.37847041, -0.61164102, -0.69473406],
            ]
        ),
    )

    a = brbd.RotoTrans.fromEulerAngles(np.array([1, 2, 3]), np.array([1, 2, 3]), "xyz")
    b = brbd.RotoTrans.fromEulerAngles(np.array([2, 3, 4]), np.array([2, 3, 4]), "xyz")
    c = a.multiply(b)
    np.testing.assert_almost_equal(
        c.to_array(),
        np.array(
            [
                [-0.38048103, -0.78702507, 0.48561892, 5.63733413],
                [-0.84379757, 0.08054074, -0.5305835, 0.109598],
                [0.37847041, -0.61164102, -0.69473406, 1.0197191],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_get_range_q(brbd)
        test_multiply(brbd)
