from collections import UserList
from typing import TYPE_CHECKING, Iterator

from .misc import BiorbdArray, to_biorbd_array_output, to_biorbd_array_input
from ..biorbd import NodeSegment as BiorbdMarker, GeneralizedCoordinates

if TYPE_CHECKING:
    from .biorbd_model import Biorbd


class Marker:
    def __init__(self, model: "Biorbd", index: int):
        self._model = model
        self._index = index

    @property
    def parent_name(self) -> str:
        """
        Get the name of the segment to which the marker is attached.

        Returns
        -------
        The name of the segment to which the marker is attached.
        """
        return self.internal.parent().to_string()

    @property
    def name(self) -> str:
        """
        Get the name of the marker.

        Returns
        -------
        The name of the marker.
        """
        return self.internal.name().to_string()

    @property
    def local(self) -> BiorbdArray:
        """
        Get the position of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The position of the marker.
        """
        return to_biorbd_array_output(self.internal)

    @local.setter
    def local(self, value: BiorbdArray):
        """
        Set the position of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: The new position of the marker.
        """
        self.internal.setValues(BiorbdMarker(to_biorbd_array_input(value)))

    @property
    def position(self) -> BiorbdArray:
        """
        Get the position of the marker in the global reference frame, assuming that the model is already in the desired position.

        Returns
        -------
        The position of the marker in the global reference frame.
        """
        dummy_q = GeneralizedCoordinates(self._model.nb_q)
        return self.forward_kinematics(q=dummy_q, update_kinematics=False)

    @property
    def x(self) -> float:
        """
        Get the x coordinate of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The x coordinate of the marker.
        """
        return self.local[0]

    @x.setter
    def x(self, value: float):
        """
        Set the x coordinate of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: float
            The x coordinate of the marker.
        """
        pos = self.local
        pos[0] = value
        self.local = pos

    @property
    def y(self) -> float:
        """
        Get the y coordinate of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The y coordinate of the marker.
        """
        return self.local[1]

    @y.setter
    def y(self, value: float):
        """
        Set the y coordinate of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: float
            The y coordinate of the marker.
        """
        pos = self.local
        pos[1] = value
        self.local = pos

    @property
    def z(self) -> float:
        """
        Get the z coordinate of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The z coordinate of the marker.
        """
        return self.local[2]

    @z.setter
    def z(self, value: float):
        """
        Set the z coordinate of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: float
            The z coordinate of the marker.
        """
        pos = self.local
        pos[2] = value
        self.local = pos

    @property
    def is_anatomical(self) -> bool:
        """
        Check if the marker is anatomical or technical.

        Returns
        -------
        True if the marker is anatomical, False if it is technical.
        """
        return self.internal.isAnatomical()

    @property
    def is_technical(self) -> bool:
        """
        Check if the marker is technical or anatomical.

        Returns
        -------
        True if the marker is technical, False if it is anatomical.
        """
        return not self.internal.isTechnical()

    def forward_kinematics(self, q: BiorbdArray, update_kinematics: bool = True) -> BiorbdArray:
        """
        Get the position of the marker in the global reference frame.

        Parameters
        ----------
        q: BiorbdArray
            The generalized coordinates of the model.
        update_kinematics: bool
            If True, the model's kinematics will be updated to reflect the new generalized coordinates.

        Returns
        -------
        The position of the marker in the global reference frame.
        """
        q = to_biorbd_array_input(q)
        return to_biorbd_array_output(self._model.internal.marker(q, self._index, update_kinematics))

    @property
    def internal(self) -> BiorbdMarker:
        """
        Get the internal marker of the Marker instance.

        Returns
        -------
        The internal marker. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model.internal.marker(self._index)


class MarkersList(UserList):
    data: list[Marker]

    def __init__(self, markers: list[Marker], model: "Biorbd"):
        super().__init__(markers)
        self._model = model

    def __getitem__(self, item: str | int) -> Marker:
        if isinstance(item, str):
            for marker in self.data:
                if marker.name == item:
                    return marker
            raise KeyError(f"Marker {item} not found")

        return self.data[item]

    def __iter__(self) -> Iterator[Marker]:
        return super().__iter__()

    def __call__(self, q: BiorbdArray) -> "MarkersList":
        """
        Perform of forward kinematics to get the position of the markers at a given pose.
        """
        q = to_biorbd_array_input(q)
        # Update the internal model and return self for convenience
        self._model.internal.markers(q)
        return self
