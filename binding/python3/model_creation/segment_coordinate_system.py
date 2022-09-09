from typing import Callable

from .axis_real import AxisReal
from .axis import Axis
from .kinematic_chain import KinematicChain
from .marker import Marker
from .protocols import Data
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class SegmentCoordinateSystem:
    def __init__(
        self,
        origin: Callable | str,
        first_axis: Axis,
        second_axis: Axis,
        axis_to_keep: AxisReal.Name,
    ):
        """
        Set the SegmentCoordinateSystemReal matrix of the segment

        Parameters
        ----------
        origin
            The function (f(m) -> np.ndarray, where m is a dict of markers (XYZ1 x time)) that defines the
            origin of the reference frame.
            If a str is provided, the position of the corresponding marker is used
        first_axis
            The first axis defining the segment_coordinate_system
        second_axis
            The second axis defining the segment_coordinate_system
        axis_to_keep
            The Axis.Name of the axis to keep while recomputing the reference frame. It must be the same as either
            first_axis.name or second_axis.name
        """

        self.origin = Marker(origin)
        self.first_axis = first_axis
        self.second_axis = second_axis
        self.axis_to_keep = axis_to_keep

    def to_scs(
            self, data: Data, kinematic_chain: KinematicChain, parent_scs: SegmentCoordinateSystemReal
    ) -> SegmentCoordinateSystemReal:
        """
        Collapse the generic SegmentCoordinateSystem to an actual SegmentCoordinateSystemReal with value
        based on the model and the data

        Parameters
        ----------
        data
            The actual data
        kinematic_chain
            The model as it is constructed at that particular time. It is useful if some values must be obtained from
            previously computed values
        parent_scs
            The SegmentCoordinateSystemReal of the parent to compute the local transformation
        Returns
        -------
        The collapsed SegmentCoordinateSystemReal
        """

        return SegmentCoordinateSystemReal.from_markers(
            self.origin.to_marker(data, kinematic_chain),
            self.first_axis.to_axis(data, kinematic_chain),
            self.second_axis.to_axis(data, kinematic_chain),
            self.axis_to_keep, parent_scs
        )
