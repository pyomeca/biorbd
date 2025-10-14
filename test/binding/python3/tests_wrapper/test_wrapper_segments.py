import numpy as np
import pytest

from wrapper_tests_utils import evaluate, brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_segments(brbd):
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")

    # Test accessors
    assert len(model.segments) == len(model.internal.segments())
    assert model.segments[0].name == "Pelvis"
    assert model.segments["Pelvis"].name == "Pelvis"

    # Test specific segment
    segment = model.segments[0]

    # Name
    assert segment.name == "Pelvis"

    # Translation and rotation sequences
    assert segment.translations == "yz"
    assert segment.rotations == "x"

    # Mass
    assert segment.mass == 9.03529
    segment.mass = 100
    assert segment.mass == 100

    # Center of mass
    np.testing.assert_almost_equal(evaluate(brbd, segment.center_of_mass), [0, 0, 0.0885])
    segment.center_of_mass = [1, 2, 3]
    np.testing.assert_almost_equal(evaluate(brbd, segment.center_of_mass), [1, 2, 3])

    # Inertia
    np.testing.assert_almost_equal(
        evaluate(brbd, segment.inertia), [[0.04664, 0.0, 0.0], [0.0, 0.07178, 0.0], [0.0, 0.0, 0.06989]]
    )
    segment.inertia = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    np.testing.assert_almost_equal(evaluate(brbd, segment.inertia), [[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # Get the markers of the segment
    assert len(segment.markers) == 6


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_wrapper_segments(brbd)
