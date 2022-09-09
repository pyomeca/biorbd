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

    def __init__(self, name: AxisReal.Name, start: Marker, end: Marker):
        """
        Parameters
        ----------
        name
            The AxisName of the Axis
        start
            The marker that defines the starting point of the axis
        end
            The marker that defines the ending point of the axis
        """
        self.name = name
        self.start = start
        self.end = end

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
