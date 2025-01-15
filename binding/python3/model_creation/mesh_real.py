from typing import Callable

import numpy as np

from .biomechanical_model_real import BiomechanicalModelReal
from .protocols import Data


class MeshReal:
    def __init__(
        self,
        positions: tuple[tuple[float, float, float], ...] = None,
    ):
        """
        Parameters
        ----------
        positions
            The 3d position of the all the mesh points
        """

        self.positions = positions

    @staticmethod
    def from_data(
        data: Data,
        functions: tuple[Callable, ...],
        kinematic_chain: BiomechanicalModelReal,
        parent_scs: "SegmentCoordinateSystemReal" = None,
    ):
        """
        This is a constructor for the MeshReal class. It evaluates the functions that defines the mesh to get
        actual positions

        Parameters
        ----------
        data
            The data to pick the data from
        functions
            The function (f(m) -> np.ndarray, where m is a dict of markers (XYZ1 x time)) that defines the mesh points
        kinematic_chain
            The model as it is constructed at that particular time. It is useful if some values must be obtained from
            previously computed values
        parent_scs
            The segment coordinate system of the parent to transform the marker from global to local
        """

        # Get the position of the all the mesh points and do some sanity checks
        all_p = []
        for f in functions:
            p: np.ndarray = f(data.values, kinematic_chain)
            if not isinstance(p, np.ndarray):
                raise RuntimeError(f"The function {f} must return a np.ndarray of dimension 4xT (XYZ1 x time)")
            if len(p.shape) == 1:
                p = p[:, np.newaxis]

            if len(p.shape) != 2 or p.shape[0] != 4:
                raise RuntimeError(f"The function {f} must return a np.ndarray of dimension 4xT (XYZ1 x time)")

            p[3, :] = 1  # Do not trust user and make sure the last value is a perfect one
            projected_p = (parent_scs.transpose if parent_scs is not None else np.identity(4)) @ p
            if np.isnan(projected_p).all():
                raise RuntimeError(f"All the values for {f} returned nan which is not permitted")

            all_p.append(projected_p)

        return MeshReal(tuple(all_p))

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = ""
        for position in self.positions:
            # Do a sanity check
            position = np.array(position)
            p = position if len(position.shape) == 1 else np.nanmean(position, axis=1)
            out_string += f"\tmesh\t{p[0]:0.4f}\t{p[1]:0.4f}\t{p[2]:0.4f}\n"
        return out_string
