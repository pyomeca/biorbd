from .inertia_parameters import InertiaParameters
from .marker import Marker
from .mesh import Mesh
from .rotations import Rotations
from .range_of_motion import RangeOfMotion, Ranges
from .segment_coordinate_system import SegmentCoordinateSystem
from .translations import Translations


class Segment:
    def __init__(
        self,
        name: str = None,
        parent_name: str = "",
        translations: Translations = Translations.NONE,
        rotations: Rotations = Rotations.NONE,
        q_ranges: RangeOfMotion = None,
        qdot_ranges: RangeOfMotion = None,
        segment_coordinate_system: SegmentCoordinateSystem = None,
        inertia_parameters: InertiaParameters = None,
        mesh: Mesh = None,
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
        inertia_parameters
            The inertia parameters of the segment
        mesh
            The mesh points of the segment
        """

        self.name = name
        self.parent_name = parent_name
        self.translations = translations
        self.rotations = rotations
        self.q_ranges = q_ranges
        self.qdot_ranges = qdot_ranges
        self.markers = []
        self.segment_coordinate_system = segment_coordinate_system
        self.inertia_parameters = inertia_parameters
        self.mesh = mesh

    def add_marker(self, marker: Marker):
        """
        Add a new marker to the segment

        Parameters
        ----------
        marker
            The marker to add
        """
        if marker.parent_name is not None and marker.parent_name != self.name:
            raise ValueError(
                "The marker name should be the same as the 'key'. Alternatively, marker.name can be left undefined"
            )

        marker.parent_name = self.name
        self.markers.append(marker)

    def add_range(self, type: Ranges, min_bound, max_bound):
        """
        Add a new rangeQ to the segment

        Parameters
        ----------
        marker
            The marker to add
        """
        if type == Ranges.Q:
            self.q_ranges = RangeOfMotion(type=type, min_bound=min_bound, max_bound=max_bound)
        elif type == Ranges.Qdot:
            self.qdot_ranges = RangeOfMotion(type=type, min_bound=min_bound, max_bound=max_bound)
        else:
            raise RuntimeError(f"add_range's type must be Ranges.Q or Ranges.Qdot (you have {type})")