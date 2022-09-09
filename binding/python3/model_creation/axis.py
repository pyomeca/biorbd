from typing import Callable

from .axis_real import AxisReal
from .marker_generic import Marker
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class Axis:
    class Name(AxisReal.Name):
        """
        A copy of AxisReal.Name
        """
        pass

    def __init__(self, name: AxisReal.Name, start: Callable | str, end: Callable | str):
        """
        Parameters
        ----------
        name
            The AxisName of the Axis
        start
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the starting point of the axis.
            If a str is provided, the position of the corresponding marker is used
        end
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the end point of the axis.
            If a str is provided, the position of the corresponding marker is used
        """
        self.name = name
        self.start = Marker(start)
        self.end = Marker(end)

    def to_axis(self, data: Data, parent_scs: SegmentCoordinateSystemReal = None) -> AxisReal:
        """
        Compute the axis from actual data
        Parameters
        ----------
        data
            The actual data
        parent_scs
            The transformation from global to local
        """

        start = self.start.to_marker(data, parent_scs)
        end = self.end.to_marker(data, parent_scs)
        return AxisReal(self.name, start, end)
