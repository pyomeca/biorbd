from .axis import Axis
from .axis_generic import AxisGeneric
from .marker_generic import MarkerGeneric
from .protocols import Data
from .rototranslation import RT


class RTGeneric:
    def __init__(self, origin: MarkerGeneric, axes: tuple[AxisGeneric, AxisGeneric, Axis.Name]):
        """
        Parameters
        ----------
        origin
            The origin of the RT
        axes
            The first (axes[0]) and second (axes[1]) axes of the RT with the name of the recomputed axis (axis[2])
        """
        self.origin = origin
        self.axes = axes

    def to_rt(self, data: Data, parent_rt: RT) -> RT:
        """
        Collapse the generic RT to an actual RT with value based on the model and the data

        Parameters
        ----------
        data
            The actual data
        parent_rt
            The RT of the parent to compute the local transformation
        Returns
        -------
        The collapsed RT
        """
        origin = self.origin.to_marker(data)
        axes = (self.axes[0].to_axis(data), self.axes[1].to_axis(data), self.axes[2])

        return RT.from_markers(origin, axes, parent_rt)
