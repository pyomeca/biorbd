from typing import Callable

import numpy as np

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
            If the marker should be flaged as a technical marker
        is_anatomical
            If the marker should be flaged as an anatomical marker
        """
        self.name = name
        self.parent_name = parent_name
        if position is None:
            position = np.array((0, 0, 0))
        self.position = position if isinstance(position, np.ndarray) else np.array(position)
        self.is_technical = is_technical
        self.is_anatomical = is_anatomical

    @staticmethod
    def from_data(
        data: Data,
        name: str,
        function: Callable,
        parent_name: str,
        parent_rt: "RT" = None,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        This is a constructor for the Marker class. It takes the mean of the position of the marker
        from the data as position

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
        parent_rt
            The RT of the parent to transform the marker from global to local
        is_technical
            If the marker should be flagged as a technical marker
        is_anatomical
            If the marker should be flagged as an anatomical marker
        """

        # Get the position of the markers and do some sanity checks
        p: np.ndarray = function(data.values)
        if not isinstance(p, np.ndarray) or np.isnan(p).any():
            raise RuntimeError(f"The function {function} must return a np.ndarray of dimension 4xT (XYZ1 x time)")
        if (len(p.shape) == 1):
            p = p[:, np.newaxis]

        if len(p.shape) != 2 or p.shape[0] != 4:
            raise RuntimeError(f"The function {function} must return a np.ndarray of dimension 4xT (XYZ1 x time)")

        mean_p = (parent_rt.transpose if parent_rt is not None else np.identity(4)) @ np.nanmean(p, axis=1)
        return MarkerReal(name, parent_name, mean_p[:3], is_technical=is_technical, is_anatomical=is_anatomical)

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"marker {self.name}\n"
        out_string += f"\tparent {self.parent_name}\n"
        out_string += f"\tposition {self.position[0]:0.4f} {self.position[1]:0.4f} {self.position[2]:0.4f}\n"
        out_string += f"\ttechnical {1 if self.is_technical else 0}\n"
        out_string += f"\tanatomical {1 if self.is_anatomical else 0}\n"
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
