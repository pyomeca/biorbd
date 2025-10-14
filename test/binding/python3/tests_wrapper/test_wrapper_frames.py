import re

import numpy as np
import pytest

from wrapper_tests_utils import evaluate, brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_frames(brbd):
    model = brbd.Biorbd("../../models/arm26.bioMod")

    # Test accessors
    assert len(model.segment_frames) == len(model.segments)
    assert model.segment_frames[2].name == model.segment_frames["r_humerus_rotation1"].name
    frame = model.segments[2].frame

    q = [0.1, 0.1]

    # Compute the reference frames of each segment at that position
    # For clarity sake, just print the first segment, in normal use, one would probably want to use all segments
    np.testing.assert_almost_equal(
        evaluate(brbd, frame.local),
        [
            [0.99750108, 0.03902081, -0.05889802, 0.0],
            [-0.03895296, 0.9992384, 0.0023, 0.0],
            [0.05894291, 0.0, 0.99826136, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    )
    frame_at_q = [
        [0.99641331, -0.06075807, -0.05889802, -0.017545],
        [0.06099902, 0.99813518, 0.0023, -0.007],
        [0.05864844, -0.00588447, 0.99826136, 0.17],
        [0.0, 0.0, 0.0, 1.0],
    ]
    if brbd.backend == brbd.CASADI:
        with pytest.raises(
            RuntimeError,
            match=re.escape(
                "The 'world' method cannot be called when using the CasADi backend. Use 'forward_kinematics' instead."
            ),
        ):
            frame.world
        with pytest.raises(
            RuntimeError, match=re.escape("The '()' accessor cannot be called when using the CasADi backend")
        ):
            frame(q)
        with pytest.raises(
            RuntimeError,
            match="The 'forward_kinematics' method without setting q cannot be called when using the CasADi backend",
        ):
            frame.forward_kinematics()
    else:
        np.testing.assert_almost_equal(frame.world, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        np.testing.assert_almost_equal(frame(q), frame_at_q)
        np.testing.assert_almost_equal(frame.world, frame_at_q)
    np.testing.assert_almost_equal(evaluate(brbd, frame.forward_kinematics, q=q), frame_at_q)

    # We can extract some useful information from the frame
    np.testing.assert_almost_equal(
        evaluate(brbd, frame.local_rotation),
        [[0.99750108, 0.03902081, -0.05889802], [-0.03895296, 0.9992384, 0.0023], [0.05894291, 0.0, 0.99826136]],
    )
    np.testing.assert_almost_equal(
        evaluate(brbd, frame.local_rotation_as_euler, angle_sequence="xyz"), [-0.002304, -0.05893213, -0.03909863]
    )
    np.testing.assert_almost_equal(evaluate(brbd, frame.local_translation), [0.0, 0.0, 0.0])
    if brbd.backend == brbd.CASADI:
        with pytest.raises(
            RuntimeError, match="The 'world_rotation' method cannot be called when using the CasADi backend"
        ):
            frame.world_rotation
        with pytest.raises(
            RuntimeError,
            match="The 'world_rotation_as_euler' method cannot be called when using the CasADi backend",
        ):
            frame.world_rotation_as_euler("xyz")
        with pytest.raises(
            RuntimeError, match="The 'world_translation' method cannot be called when using the CasADi backend"
        ):
            frame.world_translation
    else:
        np.testing.assert_almost_equal(
            frame.world_rotation,
            [
                [0.99641331, -0.06075807, -0.05889802],
                [0.06099902, 0.99813518, 0.0023],
                [0.05864844, -0.00588447, 0.99826136],
            ],
        )
        np.testing.assert_almost_equal(frame.world_rotation_as_euler("xyz"), [-0.002304, -0.05893213, 0.06090137])
        np.testing.assert_almost_equal(frame.world_translation, [-0.017545, -0.007, 0.17])


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_wrapper_frames(brbd)
