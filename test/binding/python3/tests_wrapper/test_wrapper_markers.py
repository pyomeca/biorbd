import re

import numpy as np
import pytest

from wrapper_tests_utils import evaluate, brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_markers(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    assert len(model.markers) == len(model.internal.markers())

    # Test accessors
    assert model.markers[0].name == "pelv1"
    assert model.markers["pelv1"].name == "pelv1"

    # Test a specific marker
    marker = model.markers[0]

    # Name
    assert marker.name == "pelv1"

    # Parent segment
    assert marker.segment.name == "Pelvis"

    # Position
    np.testing.assert_almost_equal(evaluate(brbd, marker.local), [-0.1038, 0.0821, 0.0])
    marker.local = [1, 2, 3]
    np.testing.assert_almost_equal(evaluate(brbd, marker.local), [1, 2, 3])

    # X, Y, Z
    assert evaluate(brbd, marker.x) == 1
    marker.x = 4
    assert evaluate(brbd, marker.x) == 4

    assert evaluate(brbd, marker.y) == 2
    marker.y = 5
    assert evaluate(brbd, marker.y) == 5

    assert evaluate(brbd, marker.z) == 3
    marker.z = 6
    assert evaluate(brbd, marker.z) == 6

    # Is technical or anatomical
    assert marker.is_anatomical is False
    assert marker.is_technical is False

    # Perform FK to get world position
    q = [0.1] * model.nb_q
    # First try without updating kinematics
    markers = model.markers
    if brbd.backend == brbd.CASADI:
        with pytest.raises(RuntimeError, match="The 'world' method cannot be called when using the CasADi backend"):
            markers[0].world
    else:
        np.testing.assert_almost_equal(markers[0].world, [4, 5, 6])

    # Then with updating kinematics
    if brbd.backend == brbd.CASADI:
        with pytest.raises(
            RuntimeError, match=re.escape("The '()' accessor cannot be called when using the CasADi backend")
        ):
            markers(q)
    else:
        markers = markers(q)
        np.testing.assert_almost_equal(markers[0].world, [4.0, 4.47602033, 6.56919207])

        # Then test that the update_kinematics is still applied
        markers = model.markers
        np.testing.assert_almost_equal(evaluate(brbd, markers[0].world), [4.0, 4.47602033, 6.56919207])

    np.testing.assert_almost_equal(evaluate(brbd, marker.forward_kinematics, q=q), [4.0, 4.47602033, 6.56919207])
    if brbd.backend == brbd.CASADI:
        with pytest.raises(
            RuntimeError,
            match="The 'forward_kinematics' method without setting q cannot be called when using the CasADi backend",
        ):
            marker.forward_kinematics()
    else:
        np.testing.assert_almost_equal(marker.forward_kinematics(), [4.0, 4.47602033, 6.56919207])

    # Test the jacobian of first marker at previous set q a set q
    jacobian_at_q = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, -6.46919207, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 4.37602033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    jacobian_at_2q = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, -6.87374612, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 3.7083169, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    if brbd.backend == brbd.CASADI:
        with pytest.raises(
            RuntimeError,
            match="The 'jacobian' method without setting q cannot be called when using the CasADi backend",
        ):
            marker.jacobian()
    else:
        np.testing.assert_almost_equal(marker.jacobian(), jacobian_at_q)
    np.testing.assert_almost_equal(evaluate(brbd, marker.jacobian, q=np.array(q) * 2), jacobian_at_2q)

    # Test the all markers jacobian
    if brbd.backend == brbd.CASADI:
        with pytest.raises(
            RuntimeError,
            match="The 'jacobian' method without setting q cannot be called when using the CasADi backend",
        ):
            markers.jacobian()
    else:
        jacobian = markers.jacobian()
        assert len(jacobian) == len(model.markers)
        np.testing.assert_almost_equal(jacobian[0], jacobian_at_2q)
    if brbd.backend == brbd.CASADI:
        # The _evaluate function flattens the output, so we need to take the first 3 columns and nb_q of last dimension as the other one does
        np.testing.assert_almost_equal(evaluate(brbd, markers.jacobian, q=q)[0, : model.nb_q], jacobian_at_q)
    else:
        np.testing.assert_almost_equal(evaluate(brbd, markers.jacobian, q=q)[0], jacobian_at_q)


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_wrapper_markers(brbd)
