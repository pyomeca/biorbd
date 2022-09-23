from .inertia_parameters_real import InertiaParametersReal
from .marker_real import MarkerReal
from .mesh_real import MeshReal
from .rotations import Rotations
from .segment_coordinate_system_real import SegmentCoordinateSystemReal
from .translations import Translations


class SegmentReal:
    def __init__(
        self,
        name: str = None,
        parent_name: str = "",
        segment_coordinate_system: SegmentCoordinateSystemReal = None,
        translations: Translations = Translations.NONE,
        rotations: str = "",
        inertia_parameters: InertiaParametersReal = None,
        mesh: MeshReal = None,
    ):
        self.name = name
        self.parent_name = parent_name
        self.translations = translations
        self.rotations = rotations
        self.markers = []
        self.segment_coordinate_system = segment_coordinate_system
        self.inertia_parameters = inertia_parameters
        self.mesh = mesh

    def add_marker(self, marker: MarkerReal):
        self.markers.append(marker)

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"segment {self.name}\n"
        if self.parent_name:
            out_string += f"\tparent {self.parent_name}\n"
        if self.segment_coordinate_system:
            out_string += f"\tRT {self.segment_coordinate_system}\n"
        if self.translations != Translations.NONE:
            out_string += f"\ttranslations {self.translations.value}\n"
        if self.rotations != Rotations.NONE:
            out_string += f"\trotations {self.rotations.value}\n"
        if self.inertia_parameters:
            out_string += str(self.inertia_parameters)
        if self.mesh:
            out_string += str(self.mesh)
        out_string += "endsegment\n"

        # Also print the markers attached to the segment
        if self.markers:
            for marker in self.markers:
                out_string += str(marker)
        return out_string
