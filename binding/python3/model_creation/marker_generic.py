from .protocols import Data

from .marker import Marker
from .rototranslation import RT


class MarkerGeneric:
    def __init__(
        self,
        name: str,
        from_markers: str | tuple[str, ...],
        parent_name: str,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        This is a pre-constructor for the Marker class. It allows to create a generic model by marker names

        Parameters
        ----------
        name:
            The name of the new marker
        from_markers:
            The name of the markers in the data
        parent_name:
            The name of the parent the marker is attached to
        is_technical
            If the marker should be flaged as a technical marker
        is_anatomical
            If the marker should be flaged as an anatomical marker
        """
        self.name = name
        self.from_markers = from_markers
        self.parent_name = parent_name
        self.is_technical = is_technical
        self.is_anatomical = is_anatomical

    def to_marker(self, data: Data, parent_rt: RT = None) -> Marker:
        return Marker.from_data(
            data,
            self.name,
            self.from_markers,
            self.parent_name,
            parent_rt,
            is_technical=self.is_technical,
            is_anatomical=self.is_anatomical,
        )

