from .axis import Axis
from .equation import Equation
from .marker_generic import MarkerGeneric
from .protocols import Data
from .rototranslation import RT


class AxisGeneric:
    def __init__(self, name: Axis.Name, equation: Equation):
        """
        Parameters
        ----------
        name
            The AxisName of the Axis
        equation
            The equation that defines the axis
        """
        self.name = name
        self.equation = equation

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

        marker_names = self.equation.get_marker_names()

        start = self.start_point.to_marker(data, parent_rt)
        end = self.end_point.to_marker(data, parent_rt)
        return Axis(self.name, start, end)
