from typing import Callable

from .kinematic_chain import KinematicChain
from .inertia_parameters_real import InertiaParametersReal
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class InertiaParameters:
    def __init__(
        self,
        relative_mass: Callable = None,
        center_of_mass: Callable = None,
        radii_of_gyration: Callable = None,
    ):
        """
        This is a pre-constructor for the InertiaParametersReal class. It allows to create a
        generic model by marker names

        Parameters
        ----------
        relative_mass
            The relative mass of the segment with respect to the full body
        center_of_mass
            The position of the center of mass from the segment coordinate system on the main axis
        radii_of_gyration
            The radius of gyration of the segment
        """
        self.relative_mass = relative_mass
        self.center_of_mass = center_of_mass
        self.radii_of_gyration = radii_of_gyration

    def to_real(
        self, data: Data, kinematic_chain: KinematicChain, parent_scs: SegmentCoordinateSystemReal = None
    ) -> InertiaParametersReal:
        return InertiaParametersReal.from_data(
            data,
            self.relative_mass,
            self.center_of_mass,
            self.radii_of_gyration,
            kinematic_chain,
            parent_scs,
        )

