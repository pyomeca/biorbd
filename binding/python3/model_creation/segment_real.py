from .marker_real import MarkerReal
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class SegmentReal:
    def __init__(
        self,
        name,
        parent_name: str = "",
        segment_coordinate_system: SegmentCoordinateSystemReal = None,
        translations: str = "",
        rotations: str = "",
        mass: float | int = 0,
        center_of_mass: tuple[tuple[int | float, int | float, int | float]] = None,
        inertia_xxyyzz: tuple[tuple[int | float, int | float, int | float]] = None,
        mesh: tuple[tuple[int | float, int | float, int | float], ...] = None,
    ):
        self.name = name
        self.parent_name = parent_name
        self.translations = translations
        self.rotations = rotations
        self.markers = []
        self.segment_coordinate_system = segment_coordinate_system
        self.mass = mass
        self.center_of_mass = center_of_mass if center_of_mass is not None else (0, 0, 0)
        self.inertia_xxyyzz = inertia_xxyyzz if inertia_xxyyzz is not None else (0, 0, 0)
        self.mesh = mesh

    def add_marker(self, marker: MarkerReal):
        self.markers.append(marker)

    def __str__(self):
        # Define the print function so it automatically format things in the file properly<
        out_string = f"segment {self.name}\n"
        if self.parent_name:
            out_string += f"\tparent {self.parent_name}\n"
        if self.segment_coordinate_system:
            out_string += f"\tRT {self.segment_coordinate_system}\n"
        if self.translations:
            out_string += f"\ttranslations {self.translations}\n"
        if self.rotations:
            out_string += f"\trotations {self.rotations}\n"
        out_string += f"\tmass {self.mass}\n"
        if self.center_of_mass:
            out_string += f"\tcom {self.center_of_mass[0]} {self.center_of_mass[1]} {self.center_of_mass[2]}\n"
        if self.inertia_xxyyzz:
            out_string += (
                f"\tinertia {self.inertia_xxyyzz[0]} 0 0\n"
                + f"\t        0 {self.inertia_xxyyzz[1]} 0\n"
                + f"\t        0 0 {self.inertia_xxyyzz[2]}\n"
            )
        if self.mesh:
            for m in self.mesh:
                out_string += f"\tmesh {m[0]} {m[1]} {m[2]}\n"
        out_string += "endsegment\n"

        # Also print the markers attached to the segment
        if self.markers:
            for marker in self.markers:
                out_string += str(marker)
        return out_string
