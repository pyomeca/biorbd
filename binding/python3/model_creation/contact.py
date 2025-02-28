from typing import Callable

from .protocols import Data
from .translations import Translations
from .contact_real import ContactReal


class Contact:
    def __init__(
        self,
        name: str,
        function: Callable | str = None,
        parent_name: str = None,
        axis: Translations = None,
    ):
        """
        Parameters
        ----------
        name
            The name of the new contact
        function
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the contact with.
        parent_name
            The name of the parent the contact is attached to
        axis
            The axis of the contact
        """
        self.name = name
        function = function if function is not None else self.name
        self.function = (lambda m, bio: m[function]) if isinstance(function, str) else function
        self.parent_name = parent_name
        self.axis = axis

    def to_contact(self, data: Data) -> ContactReal:
        return ContactReal.from_data(
            data,
            self.name,
            self.function,
            self.parent_name,
            self.axis,
        )
