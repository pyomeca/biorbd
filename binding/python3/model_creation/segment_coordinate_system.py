from .axis import Axis
from .axis_generic import AxisGeneric
from .marker_generic import MarkerGeneric
from .rototranslation_generic import RTGeneric


class SegmentCoordinateSystem:
    def __init__(
        self,
        origin_markers: str | tuple[str, ...],
        first_axis_name: Axis.Name,
        first_axis_markers: tuple[str | tuple[str, ...], str | tuple[str, ...]],
        second_axis_name: Axis.Name,
        second_axis_markers: tuple[str | tuple[str, ...], str | tuple[str, ...]],
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

        first_axis_tp = AxisGeneric(
            name=first_axis_name,
            start=MarkerGeneric(name="", from_markers=first_axis_markers[0], parent_name=""),
            end=MarkerGeneric(name="", from_markers=first_axis_markers[1], parent_name=""),
        )
        second_axis_tp = AxisGeneric(
            name=second_axis_name,
            start=MarkerGeneric(name="", from_markers=second_axis_markers[0], parent_name=""),
            end=MarkerGeneric(name="", from_markers=second_axis_markers[1], parent_name=""),
        )

        self.rt = RTGeneric(
            origin=MarkerGeneric(name="", from_markers=origin_markers, parent_name=""),
            axes=(first_axis_tp, second_axis_tp, axis_to_keep),
        )
