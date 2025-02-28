from .inertia_parameters_real import InertiaParametersReal
from .marker_real import MarkerReal
from .contact import Contact
from .mesh_real import MeshReal
from .mesh_file_real import MeshFileReal
from .rotations import Rotations
from .range_of_motion import RangeOfMotion
from .segment_coordinate_system_real import SegmentCoordinateSystemReal
from .translations import Translations


class SegmentReal:
    def __init__(
        self,
        name: str,
        parent_name: str = "",
        segment_coordinate_system: SegmentCoordinateSystemReal = None,
        translations: Translations = Translations.NONE,
        rotations: Rotations = Rotations.NONE,
        q_ranges: RangeOfMotion = None,
        qdot_ranges: RangeOfMotion = None,
        inertia_parameters: InertiaParametersReal = None,
        mesh: MeshReal = None,
        mesh_file: MeshFileReal = None,
    ):
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

    def add_marker(self, marker: MarkerReal):
        self.markers.append(marker)

    def add_contact(self, contact: Contact):
        if contact.parent_name is None:
            raise RuntimeError(f"Contacts must have parents. Contact {contact.name} does not have a parent.")
        self.contacts.append(contact)

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"segment\t{self.name}\n"
        if self.parent_name:
            out_string += f"\tparent\t{self.parent_name}\n"
        if self.segment_coordinate_system:
            out_string += f"\tRT\t{self.segment_coordinate_system}\n"
        if self.translations != Translations.NONE:
            out_string += f"\ttranslations\t{self.translations.value}\n"
        if self.rotations != Rotations.NONE:
            out_string += f"\trotations\t{self.rotations.value}\n"
        if self.q_ranges is not None:
            out_string += str(self.q_ranges)
        if self.qdot_ranges is not None:
            out_string += str(self.qdot_ranges)
        if self.inertia_parameters:
            out_string += str(self.inertia_parameters)
        if self.mesh:
            out_string += str(self.mesh)
        if self.mesh_file:
            out_string += str(self.mesh_file)
        out_string += "endsegment\n"

        # Also print the markers attached to the segment
        if self.markers:
            for marker in self.markers:
                marker.parent_name = marker.parent_name if marker.parent_name is not None else self.name
                out_string += str(marker)

        # Also print the contacts attached to the segment
        if self.contacts:
            for contact in self.contacts:
                contact.parent_name = contact.parent_name
                out_string += str(contact)

        return out_string
