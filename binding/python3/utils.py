import numpy as np


def get_range_q(biorbd_model) -> tuple[np.ndarray, np.ndarray]:
    """
    Give the ranges of generalized coordinates q

    Parameters
    ----------
    biorbd_model: biorbd.Model
        The biorbd model

    Returns
    -------
    q_range_min, q_range_max: tuple[np.ndarray, np.ndarray]
        The range min and max of the q for each dof
    """
    q_range_max = []
    q_range_min = []
    for seg in biorbd_model.segments():
        q_range_max += [q_range.max() for q_range in seg.QRanges()]
        q_range_min += [q_range.min() for q_range in seg.QRanges()]
    return np.array(q_range_min), np.array(q_range_max)

