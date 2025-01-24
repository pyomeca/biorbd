from typing import Callable

from .protocols import Data
from .via_point_real import ViaPointReal


class ViaPoint:
    def __init__(
        self,
        name: str,
        position_function: Callable | str = None,
        parent_name: str = None,
        muscle_name: str = None,
        muscle_group: str = None,
    ):
        """
        Parameters
        ----------
        name
            The name of the new via point
        position_function
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the via point with.
        parent_name
            The name of the parent the via point is attached to
        muscle_name
            The name of the muscle that passes through this via point
        muscle_group
            The muscle group the muscle belongs to
        """
        self.name = name
        position_function = position_function if position_function is not None else self.name
        self.position_function = (
            (lambda m, bio: m[position_function]) if isinstance(position_function, str) else position_function
        )
        self.parent_name = parent_name
        self.muscle_name = muscle_name
        self.muscle_group = muscle_group

    def to_via_point(self, data: Data) -> ViaPointReal:
        return ViaPointReal.from_data(
            data,
            self.name,
            self.parent_name,
            self.muscle_name,
            self.muscle_group,
            self.position_function,
        )
