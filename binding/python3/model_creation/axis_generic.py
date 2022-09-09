from .axis import Axis
from .marker_generic import MarkerGeneric
from .protocols import Data
from .rototranslation import RT


class AxisGeneric:
    def __init__(self, name: Axis.Name, start: MarkerGeneric, end: MarkerGeneric):
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

    def to_axis(self, data: Data, parent_rt: RT = None) -> Axis:
        """
        Compute the axis from actual data
        Parameters
        ----------
        data
            The actual data
        parent_rt
            The transformation from global to local
        """

        start = self.start.to_marker(data, parent_rt)
        end = self.end.to_marker(data, parent_rt)
        return Axis(self.name, start, end)
