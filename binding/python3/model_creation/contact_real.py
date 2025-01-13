from typing import Callable

import numpy as np

from .protocols import Data
from .translations import Translations

class ContactReal:
    def __init__(
        self,
        name: str,
        parent_name: str,
        position: tuple[int | float, int | float, int | float] | np.ndarray = None,
        axis: Translations = None,
    ):
        """
        Parameters
        ----------
        name
            The name of the new contact
        parent_name
            The name of the parent the contact is attached to
        position
            The 3d position of the contact
        axis
            The axis of the contact
        """
        self.name = name
        self.parent_name = parent_name
        if position is None:
            position = np.array((0, 0, 0, 1))
        self.position = position if isinstance(position, np.ndarray) else np.array(position)
        self.axis = axis

    @staticmethod
    def from_data(
        data: Data,
        name: str,
        function: Callable,
        parent_name: str,
        axis: Translations = None,
    ):
        """
        This is a constructor for the Contact class. It evaluates the function that defines the contact to get an
        actual position

        Parameters
        ----------
        data
            The data to pick the data from
        name
            The name of the new contact
        function
            The function (f(m) -> np.ndarray, where m is a dict of markers (XYZ1 x time)) that defines the contacts in the local joint coordinates.
        parent_name
            The name of the parent the contact is attached to
        axis
            The axis of the contact
        """

        # Get the position of the contact points and do some sanity checks
        p: np.ndarray = function(data.values)
        if not isinstance(p, np.ndarray):
            raise RuntimeError(f"The function {function} must return a np.ndarray of dimension 3xT (XYZ x time)")
        if len(p.shape) == 1:
            p = p[:, np.newaxis]

        if len(p.shape) != 2 or p.shape[0] != 3:
            raise RuntimeError(f"The function {function} must return a np.ndarray of dimension 3xT (XYZ x time)")

        return ContactReal(name, parent_name, p, axis)

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"contact {self.name}\n"
        out_string += f"\tparent {self.parent_name}\n"
        out_string += f"\tposition {self.position[0]:0.4f} {self.position[1]:0.4f} {self.position[2]:0.4f}\n"
        out_string += f"\taxis {self.axis.value}\n"
        out_string += "endcontact\n"
        return out_string
