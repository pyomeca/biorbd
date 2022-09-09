from .axis_real import AxisReal
from .axis import Axis
from .marker_generic import MarkerGeneric
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class SegmentCoordinateSystemGeneric:
    def __init__(
        self,
        origin: MarkerGeneric,
        first_axis: Axis,
        second_axis: Axis,
        axis_to_keep: AxisReal.Name,
    ):
        """
        Set the SegmentCoordinateSystemReal matrix of the segment

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

    def to_rt(self, data: Data, parent_rt: SegmentCoordinateSystemReal) -> SegmentCoordinateSystemReal:
        """
        Collapse the generic SegmentCoordinateSystem to an actual SegmentCoordinateSystemReal with value
        based on the model and the data

        Parameters
        ----------
        data
            The actual data
        parent_rt
            The SegmentCoordinateSystemReal of the parent to compute the local transformation
        Returns
        -------
        The collapsed SegmentCoordinateSystemReal
        """
        origin = self.origin.to_marker(data)

        return SegmentCoordinateSystemReal.from_markers(
            origin, self.first_axis.to_axis(data), self.second_axis.to_axis(data), self.axis_to_keep, parent_rt
        )
