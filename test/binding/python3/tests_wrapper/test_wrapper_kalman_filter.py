import numpy as np
import pytest

from wrapper_tests_utils import brbd_to_test


@pytest.mark.parametrize("brbd", brbd_to_test)
def test_wrapper_kalman_filter(brbd):
    if brbd.backend == brbd.CASADI:
        assert brbd.has_extended_kalman_filter is False
        return
    assert brbd.has_extended_kalman_filter is True

    # Load a predefined model
    model = brbd.Biorbd("../../models/pyomecaman.bioMod")
    n_frames = 20

    # Generate clapping gesture data
    qinit = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]
    qmid = [0, 0, -0.3, 0.5, 1.15, -0.5, 1.15, 0, 0, 0, 0, 0, 0]
    qfinal = [0, 0, -0.3, 0.35, 1.15, -0.35, 1.15, 0, 0, 0, 0, 0, 0]
    target_q = np.concatenate((np.linspace(qinit, qmid, n_frames).T, np.linspace(qmid, qfinal, n_frames).T), axis=1)
    markers = []
    for q in target_q.T:
        markers.append(np.array([mark.world for mark in model.markers(q)]).T)

    # Perform the kalman filter for each frame (remember, due to initialization, first frame is much longer than the rest)
    kalman = brbd.ExtendedKalmanFilterMarkers(model, frequency=100)
    q_recons = np.ndarray(target_q.shape)
    for i, (q_i, _, _) in enumerate(kalman.reconstruct_frames(markers)):
        q_recons[:, i] = q_i
    np.testing.assert_almost_equal(q_recons[:, -1], target_q[:, -1], decimal=6)


if __name__ == "__main__":
    for brbd in brbd_to_test:
        test_wrapper_kalman_filter(brbd)  # This test is long
