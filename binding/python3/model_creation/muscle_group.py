from typing import Callable

from .protocols import Data


class MuscleGroup:
    def __init__(
        self,
        name: str,
        origin_parent_name: str,
        insertion_parent_name: str,
    ):
        """
        Parameters
        ----------
        name
            The name of the new muscle group
        origin_parent_name
            The name of the parent segment for this muscle group
        insertion_parent_name
            The name of the insertion segment for this muscle group
        """
        # Sanity checks
        if not isinstance(name, str):
            raise ValueError("The name of the muscle group must be a string.")
        if not isinstance(origin_parent_name, str):
            raise ValueError("The name of the origin parent segment must be a string.")
        if not isinstance(insertion_parent_name, str):
            raise ValueError("The name of the insertion parent segment must be a string.")

        self.name = name
        self.origin_parent_name = origin_parent_name
        self.insertion_parent_name = insertion_parent_name

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"musclegroup\t{self.name}\n"
        out_string += f"\tOriginParent\t{self.origin_parent_name}\n"
        out_string += f"\tInsertionParent\t{self.insertion_parent_name}\n"
        out_string += "endmusclegroup\n"
        return out_string
