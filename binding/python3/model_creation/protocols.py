from typing import Protocol
import numpy as np


class Data(Protocol):
    values: dict[str, np.ndarray]  # The data in the format [marker_names, 4xT] where 3xT is the XYZ1 x time matrix


class GenericDynamicModel(Protocol):
    @property
    def segment_names(self) -> tuple[str, ...]:
        """
        Get all the name of all the segments in the dynamic model
        """
        raise NotImplementedError

    def segment_mass(self, segment: str) -> float:
        """
        Computes the mass of the requested segment

        Parameters
        ----------
        segment
            The segment to get the mass from

        Returns
        -------
        The mass of the requested segment
        """

    def segment_center_of_mass(self, segment: str, inverse_proximal: bool = False) -> tuple[float, float, float]:
        """
        Computes the position of the center of mass of the requested segment, given from the medial
        marker. If 'inverse_proximal' is set to True, then the value is returned from the distal position

        Parameters
        ----------
        segment
            The segment to get the center of mass from
        inverse_proximal
            If the value should be returned from the distal marker

        Returns
        -------
        The position of the center of mass of the requested segment, given from the medial marker
        """

    def segment_moment_of_inertia(self, segment: str) -> tuple[float, float, float]:
        """
        Computes the xx, yy, zz, (Sagittal, Transverse, Longitudinal) radii of giration

        Parameters
        ----------
        segment
            The segment to get the radii of giration from

        Returns
        -------
        The xx, yy, zz, (Sagittal, Transverse, Longitudinal) radii of giration
        """
