from copy import copy

import numpy as np

from .axis import Axis
from .marker import Marker


class RT:
    def __init__(self, rt: np.ndarray = np.identity(4), parent_rt: "RT" = None, is_rt_local: bool = False):
        """
        Parameters
        ----------
        rt
            The rt of the RT
        parent_rt
            The rt of the parent (is used when printing the model so RT is in parent's local reference frame
        is_rt_local
            If the rt is already in local reference frame
        """

        self.rt = rt
        self.parent_rt = parent_rt
        self.is_rt_local = is_rt_local

    @staticmethod
    def from_markers(origin: Marker, axes: tuple[Axis, Axis, Axis.Name], parent_rt: "RT" = None) -> "RT":
        """
        Parameters
        ----------
        origin
            The marker at the origin of the RT
        axes
            The axes that defines the RT, the AxisName is the axis to keep while constructing the RT
        parent_rt
            The rt of the parent (is used when printing the model so RT is in parent's local reference frame
        """

        # Find the two adjacent axes and reorder accordingly (assuming right-hand RT)
        first_axis = axes[0]
        second_axis = axes[1]
        axis_name_to_keep = axes[2]
        if first_axis.name == second_axis.name:
            raise ValueError("The two axes cannot be the same axis")

        if first_axis.name == Axis.Name.X:
            third_axis_name = Axis.Name.Y if second_axis.name == Axis.Name.Z else Axis.Name.Z
            if second_axis.name == Axis.Name.Z:
                first_axis, second_axis = second_axis, first_axis
        elif first_axis.name == Axis.Name.Y:
            third_axis_name = Axis.Name.Z if second_axis.name == Axis.Name.X else Axis.Name.X
            if second_axis.name == Axis.Name.X:
                first_axis, second_axis = second_axis, first_axis
        elif first_axis.name == Axis.Name.Z:
            third_axis_name = Axis.Name.X if second_axis.name == Axis.Name.Y else Axis.Name.Y
            if second_axis.name == Axis.Name.Y:
                first_axis, second_axis = second_axis, first_axis
        else:
            raise ValueError("first_axis should be an X, Y or Z axis")

        # Compute the third axis and recompute one of the previous two
        first_axis_vector = first_axis.axis()
        second_axis_vector = second_axis.axis()
        third_axis_vector = np.cross(first_axis_vector, second_axis_vector)
        if axis_name_to_keep == first_axis.name:
            second_axis_vector = np.cross(third_axis_vector, first_axis_vector)
        elif axis_name_to_keep == second_axis.name:
            first_axis_vector = np.cross(second_axis_vector, third_axis_vector)
        else:
            raise ValueError("Name of axis to keep should be one of the two axes")

        # Dispatch the result into a matrix
        rt = np.zeros((4, 4))
        rt[:3, first_axis.name.value] = first_axis_vector / np.linalg.norm(first_axis_vector)
        rt[:3, second_axis.name.value] = second_axis_vector / np.linalg.norm(second_axis_vector)
        rt[:3, third_axis_name.value] = third_axis_vector / np.linalg.norm(third_axis_vector)
        rt[:3, 3] = origin.position
        rt[3, 3] = 1

        return RT(rt=rt, parent_rt=parent_rt)

    @staticmethod
    def from_euler_and_translation(
        angles: tuple[float | int, ...],
        angle_sequence: str,
        translations: tuple[float | int, float | int, float | int],
        parent_rt: "RT" = None,
    ) -> "RT":
        """
        Construct a RT from angles and translations

        Parameters
        ----------
        angles
            The actual angles
        angle_sequence
            The angle sequence of the angles
        translations
            The XYZ translations
        parent_rt
            The rt of the parent (is used when printing the model so RT is in parent's local reference frame
        """
        matrix = {
            "x": lambda x: np.array(((1, 0, 0), (0, np.cos(x), -np.sin(x)), (0, np.sin(x), np.cos(x)))),
            "y": lambda y: np.array(((np.cos(y), 0, np.sin(y)), (0, 1, 0), (-np.sin(y), 0, np.cos(y)))),
            "z": lambda z: np.array(((np.cos(z), -np.sin(z), 0), (np.sin(z), np.cos(z), 0), (0, 0, 1))),
        }
        rt = np.identity(4)
        for angle, axis in zip(angles, angle_sequence):
            rt[:3, :3] = rt[:3, :3] @ matrix[axis](angle)
        rt[:3, 3] = translations
        return RT(rt=rt, parent_rt=parent_rt, is_rt_local=True)

    def copy(self):
        return RT(rt=copy(self.rt), parent_rt=self.parent_rt)

    def __str__(self):
        if self.is_rt_local:
            rt = self.rt
        else:
            rt = self.parent_rt.transpose @ self.rt if self.parent_rt else np.identity(4)

        tx = rt[0, 3]
        ty = rt[1, 3]
        tz = rt[2, 3]

        rx = np.arctan2(-rt[1, 2], rt[2, 2])
        ry = np.arcsin(rt[0, 2])
        rz = np.arctan2(-rt[0, 1], rt[0, 0])

        return f"{rx:0.3f} {ry:0.3f} {rz:0.3f} xyz {tx:0.3f} {ty:0.3f} {tz:0.3f}"

    def __matmul__(self, other):
        return self.rt @ other

    @property
    def transpose(self):
        out = self.copy()
        out.rt = out.rt.T
        out.rt[:3, 3] = -out.rt[:3, :3] @ out.rt[3, :3]
        out.rt[3, :3] = 0
        return out
