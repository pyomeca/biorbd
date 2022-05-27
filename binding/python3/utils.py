import biorbd
import ezc3d
import numpy as np
from ezc3d import c3d


def get_range_q(biorbd_model: biorbd.Model) -> tuple[np.ndarray, np.ndarray]:
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


def get_unit_division_factor(c3d_file: ezc3d.c3d) -> int:
    """
    Allow the users to adjust the lenght units of the c3d, the biorbd model is in meter.

    Parameters
    ----------
    c3d_file: ezc3d.c3d
        c3d file converted into an ezc3d object

    Returns
    -------
    The division factor of length units
    """
    factor_str = c3d_file["parameters"]["POINT"]["UNITS"]["value"][0]
    if factor_str == "mm":
        factor = 1000
    elif factor_str == "m":
        factor = 1
    else:
        raise NotImplementedError("This is not implemented for this unit")

    return factor
