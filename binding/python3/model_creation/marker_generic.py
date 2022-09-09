from typing import Callable

from .marker_real import MarkerReal
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class MarkerGeneric:
    def __init__(
        self,
        function: Callable,
        parent_name: str = "",
        name: str = None,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        This is a pre-constructor for the Marker class. It allows to create a generic model by marker names

        Parameters
        ----------
        function
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the marker with
        parent_name
            The name of the parent the marker is attached to
        name
            The name of the new marker
        is_technical
            If the marker should be flagged as a technical marker
        is_anatomical
            If the marker should be flagged as an anatomical marker
        """
        self.name = name
        self.function = function
        self.parent_name = parent_name
        self.is_technical = is_technical
        self.is_anatomical = is_anatomical

    def to_marker(self, data: Data, parent_rt: SegmentCoordinateSystemReal = None) -> MarkerReal:
        return MarkerReal.from_data(
            data,
            self.name,
            self.function,
            self.parent_name,
            parent_rt,
            is_technical=self.is_technical,
            is_anatomical=self.is_anatomical,
        )

