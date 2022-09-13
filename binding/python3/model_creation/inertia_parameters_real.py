from typing import Callable

import numpy as np

from .kinematic_chain import KinematicChain
from .protocols import Data


class InertiaParametersReal:
    def __init__(
        self,
        mass: float = None,
        center_of_mass: np.ndarray = None,
        radii_of_gyration: np.ndarray = None,
    ):
        """
        Parameters
        ----------
        mass
            The mass of the segment
        center_of_mass
            The position of the center of mass in its parent segment reference frame
        radii_of_gyration
            The radius of gyration of the segment
        """
        self.mass = mass
        self.center_of_mass = center_of_mass
        self.radii_of_gyration = radii_of_gyration

    @staticmethod
    def from_data(
        data: Data,
        relative_mass: Callable,
        center_of_mass: Callable,
        radii_of_gyration: Callable,
        kinematic_chain: KinematicChain,
        parent_scs: "SegmentCoordinateSystemReal" = None,
    ):
        """
        This is a constructor for the InertiaParameterReal class.

        Parameters
        ----------
        data
            The data to pick the data from
        relative_mass
            The relative_mass of the segment
        center_of_mass
            The position of the center of mass from the segment coordinate system on the main axis
        radii_of_gyration
            The radius of gyration of the segment
        kinematic_chain
            The model as it is constructed at that particular time. It is useful if some values must be obtained from
            previously computed values
        parent_scs
            The segment coordinate system of the parent to transform the marker from global to local
        """

        mass = relative_mass(data.values, kinematic_chain)

        p: np.ndarray = center_of_mass(data.values, kinematic_chain)
        if not isinstance(p, np.ndarray):
            raise RuntimeError(f"The function {center_of_mass} must return a np.ndarray of dimension 4xT (XYZ1 x time)")
        if len(p.shape) == 1:
            p = p[:, np.newaxis]

        if len(p.shape) != 2 or p.shape[0] != 4:
            raise RuntimeError(f"The function {center_of_mass} must return a np.ndarray of dimension 4xT (XYZ1 x time)")

        p[3, :] = 1  # Do not trust user and make sure the last value is a perfect one
        com = (parent_scs.transpose if parent_scs is not None else np.identity(4)) @ p
        if np.isnan(com).all():
            raise RuntimeError(f"All the values for {com} returned nan which is not permitted")

        radii_of_gyration: np.ndarray = radii_of_gyration(data.values, kinematic_chain)

        return InertiaParametersReal(mass, com, radii_of_gyration)

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        com = np.nanmean(self.center_of_mass, axis=1)[:3]
        radii = self.radii_of_gyration

        out_string = f"\tmass {self.mass}\n"
        out_string += f"\tCenterOfMass {com[0]:0.5f} {com[1]:0.5f} {com[2]:0.5f}\n"
        out_string += f"\tRadiiOfGyration {radii[0]} {radii[1]} {radii[2]}\n"
        return out_string
