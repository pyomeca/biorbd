from .misc import BiorbdArray, to_biorbd_array_output, to_biorbd_array_input
from ..biorbd import NodeSegment as BiorbdMarker


class Marker:
    def __init__(self, marker: BiorbdMarker):
        self._marker = marker

    @property
    def parent_name(self) -> str:
        """
        Get the name of the segment to which the marker is attached.

        Returns
        -------
        The name of the segment to which the marker is attached.
        """
        return self._marker.parent().to_string()

    @property
    def name(self) -> str:
        """
        Get the name of the marker.

        Returns
        -------
        The name of the marker.
        """
        return self._marker.name().to_string()

    @property
    def position(self) -> BiorbdArray:
        """
        Get the position of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The position of the marker.
        """
        return to_biorbd_array_output(self._marker)

    @position.setter
    def position(self, new_position: BiorbdArray):
        """
        Set the position of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        new_position: The new position of the marker.
        """
        self._marker.setPosition(to_biorbd_array_input(new_position))

    @property
    def x(self) -> float:
        """
        Get the x coordinate of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The x coordinate of the marker.
        """
        return self.position[0]

    @x.setter
    def x(self, value: float):
        """
        Set the x coordinate of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: float
            The x coordinate of the marker.
        """
        pos = self.position
        pos[0] = value
        self.position = pos

    @property
    def y(self) -> float:
        """
        Get the y coordinate of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The y coordinate of the marker.
        """
        return self.position[1]

    @y.setter
    def y(self, value: float):
        """
        Set the y coordinate of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: float
            The y coordinate of the marker.
        """
        pos = self.position
        pos[1] = value
        self.position = pos

    @property
    def z(self) -> float:
        """
        Get the z coordinate of the marker in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The z coordinate of the marker.
        """
        return self.position[2]

    @z.setter
    def z(self, value: float):
        """
        Set the z coordinate of the marker in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: float
            The z coordinate of the marker.
        """
        pos = self.position
        pos[2] = value
        self.position = pos

    @property
    def is_anatomical(self) -> bool:
        """
        Check if the marker is anatomical or technical.

        Returns
        -------
        True if the marker is anatomical, False if it is technical.
        """
        return self._marker.isAnatomical()

    @property
    def is_technical(self) -> bool:
        """
        Check if the marker is technical or anatomical.

        Returns
        -------
        True if the marker is technical, False if it is anatomical.
        """
        return not self._marker.isTechnical()

    @property
    def internal(self) -> BiorbdMarker:
        """
        Get the internal marker of the Marker instance.

        Returns
        -------
        The internal marker. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._marker


class GlobalMarker(Marker):
    @property
    def position(self) -> BiorbdArray:
        return super().position

    @position.setter
    def position(self, new_position: BiorbdArray):
        raise RuntimeError("You cannot set the position of a global marker.")

    @property
    def x(self) -> float:
        return super().x

    @x.setter
    def x(self, value: float):
        raise RuntimeError("You cannot set the x coordinate of a global marker.")

    @property
    def y(self) -> float:
        return super().y

    @y.setter
    def y(self, value: float):
        raise RuntimeError("You cannot set the y coordinate of a global marker.")

    @property
    def z(self) -> float:
        return super().z

    def z(self, value: float):
        raise RuntimeError("You cannot set the z coordinate of a global marker.")
