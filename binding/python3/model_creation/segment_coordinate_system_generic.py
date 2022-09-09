from .axis_real import AxisReal
from .axis_generic import AxisGeneric
from .marker_generic import MarkerGeneric
from .protocols import Data
from .rototranslation import RT


class SegmentCoordinateSystemGeneric:
    def __init__(
        self,
        origin: MarkerGeneric,
        first_axis: AxisGeneric,
        second_axis: AxisGeneric,
        axis_to_keep: AxisReal.Name,
    ):
        """
        Set the RT matrix of the segment

        Parameters
        ----------
        origin
            The function (f(m) -> np.ndarray, where m is a dict of markers (XYZ1 x time)) that defines the origin of the reference
            frame.
        first_axis
            The first axis defining the segment_coordinate_system
        second_axis
            The second axis defining the segment_coordinate_system
        axis_to_keep
            The Axis.Name of the axis to keep while recomputing the reference frame. It must be the same as either
            first_axis.name or second_axis.name
        """

        self.origin = origin
        self.first_axis = first_axis
        self.second_axis = second_axis
        self.axis_to_keep = axis_to_keep

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

        return RT.from_markers(
            origin, self.first_axis.to_axis(data), self.second_axis.to_axis(data), self.axis_to_keep, parent_rt
        )
