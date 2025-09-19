from typing import TYPE_CHECKING

from .misc import BiorbdArray, to_biorbd_array_input, to_biorbd_array_output
from ..biorbd import Segment as BiorbdSegment, Characteristics as BiorbdSegmentCharacteristics

if TYPE_CHECKING:
    from .biorbd_model import Biorbd


class Segment:
    def __init__(self, model: "Biorbd", index: int):
        self._model = model
        self._index = index

    @property
    def translations(self) -> str:
        """
        Get the translations sequence of the segment.

        Returns
        -------
        The translations of the segment.
        """
        return self.internal.seqT().to_string()

    @property
    def rotations(self) -> str:
        """
        Get the rotations sequence of the segment.

        Returns
        -------
        The rotations of the segment.
        """
        return self.internal.seqR().to_string()

    @property
    def name(self) -> str:
        """
        Get the name of the segment.

        Returns
        -------
        The name of the segment.
        """
        return self.internal.name().to_string()

    @property
    def mass(self) -> float:
        """
        Get the mass of the segment.

        Returns
        -------
        The mass of the segment.
        """
        return self._characteristics.mass()

    @mass.setter
    def mass(self, value: float):
        """
        Set the mass of the segment.

        Parameters
        ----------
        value: float
            The mass of the segment.
        """
        self._characteristics.setMass(value)

    @property
    def center_of_mass(self) -> BiorbdArray:
        """
        Get the center of mass of the segment.

        Returns
        -------
        The center of mass of the segment.
        """
        return to_biorbd_array_output(self._characteristics.CoM())

    @center_of_mass.setter
    def center_of_mass(self, value: BiorbdArray):
        """
        Set the center of mass of the segment.

        Parameters
        ----------
        value: BiorbdArray
            The center of mass of the segment.
        """
        self._characteristics.setCoM(to_biorbd_array_input(value))

    @property
    def inertia(self) -> BiorbdArray:
        """
        Get the inertia of the segment.

        Returns
        -------
        The inertia of the segment.
        """
        return to_biorbd_array_output(self._characteristics.inertia())

    @inertia.setter
    def inertia(self, value: BiorbdArray):
        """
        Set the inertia of the segment.

        Parameters
        ----------
        value: BiorbdArray
            The inertia of the segment.
        """
        self._characteristics.setInertia(to_biorbd_array_input(value))

    @property
    def internal(self) -> BiorbdSegment:
        """
        Get the internal segment of the Segment instance.

        Returns
        -------
        The internal segment. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model.internal.segment(self._index)

    @property
    def _characteristics(self) -> BiorbdSegmentCharacteristics:
        return self.internal.characteristics()
