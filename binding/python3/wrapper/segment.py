from collections import UserList
from typing import TYPE_CHECKING, Iterator

from .misc import BiorbdArray, BiorbdScalar, to_biorbd_array_input, to_biorbd_array_output
from .marker import MarkersList
from .segment_frame import SegmentFrame
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
    def mass(self) -> BiorbdScalar:
        """
        Get the mass of the segment.

        Returns
        -------
        The mass of the segment.
        """
        return self._characteristics.mass()

    @mass.setter
    def mass(self, value: BiorbdScalar):
        """
        Set the mass of the segment.

        Parameters
        ----------
        value: BiorbdScalar
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
    def markers(self) -> MarkersList:
        """
        Get the markers attached to the segment.

        Returns
        -------
        The markers attached to the segment.
        """
        return MarkersList(
            [marker for marker in self._model.markers if marker.segment.name == self.name], model=self._model
        )

    @property
    def frame(self) -> SegmentFrame:
        """
        Get the segment frame.

        Returns
        -------
        The segment frame.
        """
        return SegmentFrame(self._model, self._index)

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


class SegmentsList(UserList):
    data: list[Segment]

    def __getitem__(self, item: str | int) -> Segment:
        if isinstance(item, str):
            for seg in self.data:
                if seg.name == item:
                    return seg
            raise KeyError(f"Segment {item} not found")

        return self.data[item]

    def __iter__(self) -> Iterator[Segment]:
        return super().__iter__()
