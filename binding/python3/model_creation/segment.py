from .inertia_parameters import InertiaParameters
from .marker import Marker
from .contact import Contact
from .mesh import Mesh
from .mesh_file import MeshFile
from .rotations import Rotations
from .range_of_motion import RangeOfMotion, Ranges
from .segment_coordinate_system import SegmentCoordinateSystem
from .translations import Translations


class Segment:
    def __init__(
        self,
        name,
        parent_name: str = "",
        translations: Translations = Translations.NONE,
        rotations: Rotations = Rotations.NONE,
        q_ranges: RangeOfMotion = None,
        qdot_ranges: RangeOfMotion = None,
        segment_coordinate_system: SegmentCoordinateSystem = None,
        inertia_parameters: InertiaParameters = None,
        mesh: Mesh = None,
        mesh_file: MeshFile = None,
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
        q_ranges
            The range of motion of the segment
        qdot_ranges
            The range of motion of the segment
        segment_coordinate_system
            The coordinate system of the segment
        inertia_parameters
            The inertia parameters of the segment
        mesh
            The mesh points of the segment
        mesh_file
            The mesh file of the segment
        """

        self.name = name
        self.parent_name = parent_name
        self.translations = translations
        self.rotations = rotations
        self.q_ranges = q_ranges
        self.qdot_ranges = qdot_ranges
        self.markers = []
        self.contacts = []
        self.segment_coordinate_system = segment_coordinate_system
        self.inertia_parameters = inertia_parameters
        self.mesh = mesh
        self.mesh_file = mesh_file

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

    def add_contact(self, contact: Contact):
        """
        Add a new contact to the segment

        Parameters
        ----------
        contact
            The contact to add
        """
        if contact.parent_name is None:
            raise ValueError(f"Contacts must have parents. Contact {contact.name} does not have a parent.")
        elif contact.parent_name != self.name:
            raise ValueError("The contact name should be the same as the 'key'.")
        contact.parent_name = self.name
        self.contacts.append(contact)

    def add_range(self, range_type: Ranges, min_bound, max_bound):
        """
        Add a new rangeQ to the segment

        Parameters
        ----------
        marker
            The marker to add
        """
        if range_type == Ranges.Q:
            self.q_ranges = RangeOfMotion(range_type=range_type, min_bound=min_bound, max_bound=max_bound)
        elif range_type == Ranges.Qdot:
            self.qdot_ranges = RangeOfMotion(range_type=range_type, min_bound=min_bound, max_bound=max_bound)
        else:
            raise RuntimeError(f"add_range's range_type must be Ranges.Q or Ranges.Qdot (you have {range_type})")
