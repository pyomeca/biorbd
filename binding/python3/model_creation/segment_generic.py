from .marker_generic import MarkerGeneric
from .segment_coordinate_system_generic import SegmentCoordinateSystemGeneric


class SegmentGeneric:
    def __init__(
        self,
        name,
        parent_name: str = "",
        translations: str = "",
        rotations: str = "",
        segment_coordinate_system: SegmentCoordinateSystemGeneric = None,
    ):
        """
        Create a new generic segment.

        Parameters
        ----------
        name
            The name of the segment
        parent_name
            The name of the segment the current segment is attached to
        translations
            The sequence of translation
        rotations
            The sequence of rotation
        """

        self.name = name
        self.parent_name = parent_name
        self.translations = translations
        self.rotations = rotations
        self.markers = []
        self.segment_coordinate_system = segment_coordinate_system

    def add_marker(self, marker: MarkerGeneric):
        """
        Add a new marker to the segment

        Parameters
        ----------
        marker
            The marker to add
        """
        self.markers.append(marker)
