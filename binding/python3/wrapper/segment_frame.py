from collections import UserList
from typing import TYPE_CHECKING, Iterator

from .misc import BiorbdArray, to_biorbd_array_output, to_biorbd_array_input

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
    from .segment import Segment
from ..biorbd import GeneralizedCoordinates, RotoTrans, Rotation


class SegmentFrame:
    def __init__(self, model: "Biorbd", index: int):
        self._model = model
        self._index = index

    @property
    def name(self) -> str:
        """
        Get the name of the segment to which the frame is attached.

        Returns
        -------
        The name of the segment to which the frame is attached.
        """
        return self.parent.name

    @property
    def parent(self) -> "Segment":
        """
        Get the segment to which the frame is attached.

        Returns
        -------
        The segment to which the frame is attached.
        """
        return self._model.segments[self._index]

    @property
    def local(self) -> BiorbdArray:
        """
        Get the position of the frame in the local reference frame of the segment to which it is attached.

        Returns
        -------
        The position of the frame.
        """
        return to_biorbd_array_output(self.internal)

    @local.setter
    def local(self, value: BiorbdArray):
        """
        Set the position of the frame in the local reference frame of the segment to which it is attached.

        Parameters
        ----------
        value: The new position of the frame.
        """
        new_frame = to_biorbd_array_input(value)
        if new_frame.shape == (3, 3):
            new_frame = [
                [new_frame[0, 0], new_frame[0, 1], new_frame[0, 2], 0],
                [new_frame[1, 0], new_frame[1, 1], new_frame[1, 2], 0],
                [new_frame[2, 0], new_frame[2, 1], new_frame[2, 2], 0],
                [0, 0, 0, 1],
            ]
        if new_frame.shape != (4, 4):
            raise ValueError("The new frame must be a 3x3 or 4x4 matrix")

        raise NotImplementedError("Setting a new local frame is not yet implemented in the C++ core")

    @property
    def local_rotation(self) -> BiorbdArray:
        """
        Get the rotation matrix part of the segment frame.

        Returns
        -------
        The rotation matrix of the segment frame.
        """
        return to_biorbd_array_output(self.internal.rot())

    def local_rotation_as_euler(self, angle_sequence: str = "xyz") -> BiorbdArray:
        """
        Get the rotation matrix part of the segment frame as Euler angles.

        Parameters
        ----------
        angle_sequence: str
            The sequence of the Euler angles, that is any combination of 1, 2 or 3 different axes. Default is "xyz".

        Returns
        -------
        The rotation matrix of the segment frame as Euler angles.
        """

        return to_biorbd_array_output(Rotation.toEulerAngles(self.internal.rot(), angle_sequence))

    @property
    def local_translation(self) -> BiorbdArray:
        """
        Get the translation vector part of the segment frame.

        Returns
        -------
        The translation vector of the segment frame.
        """
        return to_biorbd_array_output(self.internal.trans())

    def __call__(self, q: BiorbdArray | None = None) -> BiorbdArray:
        """
        Alias for SegmentFrame.forward_kinematics

        Parameters
        ----------
        q: BiorbdArray
            The generalized coordinates of the model.
        update_kinematics: bool
            If True, the model's kinematics will be updated to reflect the new generalized coordinates.

        Returns
        -------
        The position of the frame in the world frame.
        """
        return self.forward_kinematics(q)

    @property
    def world(self) -> BiorbdArray:
        """
        Get the position of the frame in the world reference frame evaluated at the current model's kinematics.

        Returns
        -------
        The position of the frame in the world reference frame.
        """
        return self.forward_kinematics()

    @property
    def world_rotation(self) -> BiorbdArray:
        """
        Get the rotation matrix part of the segment frame in the world reference frame evaluated at the current model's kinematics.

        Returns
        -------
        The rotation matrix of the segment frame in the world reference frame.
        """
        return to_biorbd_array_output(self._forward_kinematics().rot())

    def world_rotation_as_euler(self, angle_sequence: str = "xyz") -> BiorbdArray:
        """
        Get the rotation matrix part of the segment frame in the world reference frame as Euler angles evaluated at the
        current model's kinematics.

        Parameters
        ----------
        angle_sequence: str
            The sequence of the Euler angles, that is any combination of 1, 2 or 3 different axes. Default is "xyz".

        Returns
        -------
        The rotation matrix of the segment frame in the world reference frame as Euler angles.
        """

        return to_biorbd_array_output(Rotation.toEulerAngles(self._forward_kinematics().rot(), angle_sequence))

    @property
    def world_translation(self) -> BiorbdArray:
        """
        Get the translation vector part of the segment frame in the world reference frame evaluated at the current model's kinematics.

        Returns
        -------
        The translation vector of the segment frame in the world reference frame.
        """
        return to_biorbd_array_output(self._forward_kinematics().trans())

    def forward_kinematics(self, q: BiorbdArray | None = None) -> BiorbdArray:
        """
        Get the position of the frame in the world frame.

        Parameters
        ----------
        q: BiorbdArray
            The generalized coordinates of the model.
        update_kinematics: bool
            If True, the model's kinematics will be updated to reflect the new generalized coordinates.

        Returns
        -------
        The position of the frame in the world frame.
        """

        return to_biorbd_array_output(self._forward_kinematics(q))

    def _forward_kinematics(self, q: BiorbdArray | None = None) -> RotoTrans:
        """
        Compute the forward kinematics of the segment frame for internal purposes.

        Parameters
        ----------
        q: BiorbdArray
            The generalized coordinates of the model.
        update_kinematics: bool
            If True, the model's kinematics will be updated to reflect the new generalized coordinates.

        Returns
        -------
        The position of the frame in the world frame.
        """

        self._model.update_kinematics(q)
        update_kinematics = False
        dummy_q = GeneralizedCoordinates(self._model.nb_q)
        return self._model.internal.globalJCS(dummy_q, self._index, update_kinematics)

    @property
    def internal(self) -> RotoTrans:
        """
        Get the internal frame of the RotoTrans instance.

        Returns
        -------
        The internal frame. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model.internal.localJCS(self._index)


class SegmentFramesList(UserList):
    data: list[SegmentFrame]

    def __init__(self, frames: list[SegmentFrame], model: "Biorbd"):
        super().__init__(frames)
        self._model = model

    def __getitem__(self, item: str | int) -> SegmentFrame:
        if isinstance(item, str):
            for frame in self.data:
                if frame.parent.name == item:
                    return frame
            raise KeyError(f"Frame for segment {item} not found")

        return self.data[item]

    def __iter__(self) -> Iterator[SegmentFrame]:
        return super().__iter__()

    def __call__(self, q: BiorbdArray | None = None) -> "SegmentFramesList":
        """
        Perform of forward kinematics to get the position of the reference frames at a given pose.
        """
        # Update the internal model and return self for convenience
        self._model.update_kinematics(q)
        return self
