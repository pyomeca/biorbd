from typing import Callable

import numpy as np

from .biomechanical_model_real import BiomechanicalModelReal
from .protocols import Data


class MarkerReal:
    def __init__(
        self,
        name: str,
        parent_name: str,
        position: tuple[int | float, int | float, int | float] | np.ndarray = None,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        Parameters
        ----------
        name
            The name of the new marker
        parent_name
            The name of the parent the marker is attached to
        position
            The 3d position of the marker
        is_technical
            If the marker should be flagged as a technical marker
        is_anatomical
            If the marker should be flagged as an anatomical marker
        """
        self.name = name
        self.parent_name = parent_name
        if position is None:
            position = np.array((0, 0, 0, 1))
        self.position = position if isinstance(position, np.ndarray) else np.array(position)
        self.is_technical = is_technical
        self.is_anatomical = is_anatomical

    @staticmethod
    def from_data(
        data: Data,
        name: str,
        function: Callable,
        parent_name: str,
        kinematic_chain: BiomechanicalModelReal,
        parent_scs: "SegmentCoordinateSystemReal" = None,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        This is a constructor for the MarkerReal class. It evaluates the function that defines the marker to get an
        actual position

        Parameters
        ----------
        data
            The data to pick the data from
        name
            The name of the new marker
        function
            The function (f(m) -> np.ndarray, where m is a dict of markers (XYZ1 x time)) that defines the marker
        parent_name
            The name of the parent the marker is attached to
        kinematic_chain
            The model as it is constructed at that particular time. It is useful if some values must be obtained from
            previously computed values
        parent_scs
            The segment coordinate system of the parent to transform the marker from global to local
        is_technical
            If the marker should be flagged as a technical marker
        is_anatomical
            If the marker should be flagged as an anatomical marker
        """

        # Get the position of the markers and do some sanity checks
        p: np.ndarray = function(data.values, kinematic_chain)
        if not isinstance(p, np.ndarray):
            raise RuntimeError(f"The function {function} must return a np.ndarray of dimension 4xT (XYZ1 x time)")
        if len(p.shape) == 1:
            p = p[:, np.newaxis]

        if len(p.shape) != 2 or p.shape[0] != 4:
            raise RuntimeError(f"The function {function} must return a np.ndarray of dimension 4xT (XYZ1 x time)")

        p[3, :] = 1  # Do not trust user and make sure the last value is a perfect one
        projected_p = (parent_scs.transpose if parent_scs is not None else np.identity(4)) @ p
        if np.isnan(projected_p).all():
            raise RuntimeError(f"All the values for {function} returned nan which is not permitted")
        return MarkerReal(name, parent_name, projected_p, is_technical=is_technical, is_anatomical=is_anatomical)

    @property
    def mean_position(self) -> np.ndarray:
        """
        Get the mean value of the marker position
        """
        p = np.array(self.position)
        p = p if len(p.shape) == 1 else np.nanmean(p, axis=1)
        p = p if len(p.shape) == 1 else np.nanmean(p, axis=0)
        return p

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"marker\t{self.name}\n"
        out_string += f"\tparent\t{self.parent_name}\n"

        p = self.mean_position
        out_string += f"\tposition\t{p[0]:0.4f}\t{p[1]:0.4f}\t{p[2]:0.4f}\n"
        out_string += f"\ttechnical\t{1 if self.is_technical else 0}\n"
        out_string += f"\tanatomical\t{1 if self.is_anatomical else 0}\n"
        out_string += "endmarker\n"
        return out_string

    def __add__(self, other: np.ndarray | tuple):
        if isinstance(other, tuple):
            other = np.array(other)

        if isinstance(other, np.ndarray):
            return MarkerReal(name=self.name, parent_name=self.parent_name, position=self.position + other)
        elif isinstance(other, MarkerReal):
            return MarkerReal(name=self.name, parent_name=self.parent_name, position=self.position + other.position)
        else:
            raise NotImplementedError(f"The addition for {type(other)} is not implemented")

    def __sub__(self, other):
        if isinstance(other, tuple):
            other = np.array(other)

        if isinstance(other, np.ndarray):
            return MarkerReal(name=self.name, parent_name=self.parent_name, position=self.position - other)
        elif isinstance(other, MarkerReal):
            return MarkerReal(name=self.name, parent_name=self.parent_name, position=self.position - other.position)
        else:
            raise NotImplementedError(f"The subtraction for {type(other)} is not implemented")
