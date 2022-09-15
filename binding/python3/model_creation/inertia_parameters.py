from typing import Callable

import numpy as np

from .biomechanical_model_real import BiomechanicalModelReal
from .inertia_parameters_real import InertiaParametersReal
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class InertiaParameters:
    def __init__(
        self,
        mass: Callable = None,
        center_of_mass: Callable = None,
        inertia: Callable = None,
    ):
        """
        This is a pre-constructor for the InertiaParametersReal class. It allows to create a
        generic model by marker names

        Parameters
        ----------
        mass
            The callback function that returns the mass of the segment with respect to the full body
        center_of_mass
            The callback function that returns the position of the center of mass
            from the segment coordinate system on the main axis
        inertia
            The callback function that returns the inertia xx, yy and zz parameters of the segment
        """
        self.relative_mass = mass
        self.center_of_mass = center_of_mass
        self.inertia = inertia

    def to_real(
        self, data: Data, kinematic_chain: BiomechanicalModelReal, parent_scs: SegmentCoordinateSystemReal = None
    ) -> InertiaParametersReal:
        return InertiaParametersReal.from_data(
            data,
            self.relative_mass,
            self.center_of_mass,
            self.inertia,
            kinematic_chain,
            parent_scs,
        )

    @staticmethod
    def radii_of_gyration_to_inertia(
        mass: float, coef: tuple[float, float, float], start: np.ndarray, end: np.ndarray
    ) -> np.ndarray:
        """
        Computes the xx, yy and zz values of the matrix of inertia from the segment length. The radii of gyration used are
        'coef * length', where length is '||end - start||'

        Parameters
        ----------
        mass
            The mass of the segment
        coef
            The coefficient of the length of the segment that gives the radius of gyration about x, y and z
        start
            The starting point of the segment
        end
            The end point of the segment

        Returns
        -------
        The xx, yy, zz values of the matrix of inertia
        """

        if len(start.shape) == 1:
            start = start[:, np.newaxis]
        if len(end.shape) == 1:
            end = end[:, np.newaxis]

        length = np.nanmean(np.linalg.norm(end[:3, :] - start[:3, :], axis=0))
        r_2 = (np.array(coef) * length) ** 2
        return mass * r_2
