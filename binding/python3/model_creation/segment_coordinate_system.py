from .axis import Axis
from .axis_generic import AxisGeneric
from .equation import Equation
from .marker_generic import MarkerGeneric
from .rototranslation_generic import RTGeneric


class SegmentCoordinateSystem:
    def __init__(
        self,
        origin: Equation,
        first_axis: tuple[Axis.Name, Equation],
        second_axis: tuple[Axis.Name, Equation],
        axis_to_keep: Axis.Name,
    ):
        """
        Set the RT matrix of the segment

        Parameters
        ----------
        origin_markers
            The name of the marker to the origin of the reference frame. If multiple names are provided, the mean of
            all the given markers is used
        first_axis_name
            The Axis.Name of the first axis
        first_axis_markers
            The name of the markers that constitute the starting (first_axis_markers[0]) and
            ending (first_axis_markers[1]) of the first axis vector. Both [0] and [1] can be from multiple markers.
            If it is the case the mean of all the given markers is used
        second_axis_name
            The Axis.Name of the second axis
        second_axis_markers
            The name of the markers that constitute the starting (second_axis_markers[0]) and
            ending (second_axis_markers[1]) of the second axis vector. Both [0] and [1] can be from multiple markers.
            If it is the case the mean of all the given markers is used
        axis_to_keep
            The Axis.Name of the axis to keep while recomputing the reference frame. It must be the same as either
            first_axis_name or second_axis_name
        """

        first_axis_tp = AxisGeneric(name=first_axis[0], equation=first_axis[1])
        second_axis_tp = AxisGeneric(name=second_axis[0], equation=second_axis[1])

        self.rt = RTGeneric(
            origin=MarkerGeneric(equation=origin, parent_name=""),
            axes=(first_axis_tp, second_axis_tp, axis_to_keep),
        )
