from enum import Enum
from copy import copy

import numpy as np
import biorbd

ezc3d_found = True
try:
    import ezc3d
except ModuleNotFoundError:
    ezc3d_found = False
    ezc3d = None


class Marker:
    def __init__(
        self,
        name: str,
        parent_name: str,
        position: tuple[int | float, int | float, int | float] | np.ndarray = None,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        Parameters
        ----------
        name
            The name of the new marker
        parent_name
            The name of the parent the marker is attached to
        position
            The 3d position of the marker
        is_technical
            If the marker should be flaged as a technical marker
        is_anatomical
            If the marker should be flaged as an anatomical marker
        """
        self.name = name
        self.parent_name = parent_name
        if position is None:
            position = np.array((0, 0, 0))
        self.position = position if isinstance(position, np.ndarray) else np.array(position)
        self.is_technical = is_technical
        self.is_anatomical = is_anatomical

    @staticmethod
    def from_data(c3d: ezc3d, name: str, from_markers: str | tuple[str, ...], parent_name: str, parent_rt: "RT" = None, is_technical: bool = True, is_anatomical: bool = False):
        """
        This is a constructor for the Marker class. It takes the mean of the position of the marker
        from the c3d as position
        Parameters
        ----------
        c3d
            The data to pick the data from
        name
            The name of the new marker
        from_markers
            The name of the markers in the data
        parent_name
            The name of the parent the marker is attached to
        parent_rt
            The RT of the parent to transform the marker from global to local
        is_technical
            If the marker should be flaged as a technical marker
        is_anatomical
            If the marker should be flaged as an anatomical marker
        """

        if not ezc3d_found:
            raise RuntimeError("Ezc3d must be install to use the 'from_data' constructor")
        if isinstance(from_markers, str):
            from_markers = (from_markers,)

        def indices_in_c3d() -> tuple[int, ...]:
            return tuple(c3d["parameters"]["POINT"]["LABELS"]["value"].index(n) for n in from_markers)

        def to_meter(data: np.array) -> np.array:
            units = c3d["parameters"]["POINT"]["UNITS"]["value"]
            factor = 1000 if len(units) > 0 and units[0] == "mm" else 1

            data /= factor
            data[3] = 1
            return data

        index = indices_in_c3d()
        position = to_meter(np.mean(np.nanmean(c3d["data"]["points"][:, index, :], axis=2), axis=1))
        if np.isnan(position).any():
            raise RuntimeError(f"The markers {from_markers} are not present in the c3d")
        position = (parent_rt.transpose if parent_rt is not None else np.identity(4)) @ position
        return Marker(name, parent_name, position[:3], is_technical=is_technical, is_anatomical=is_anatomical)

    def __str__(self):
        # Define the print function so it automatically format things in the file properly
        out_string = f"marker {self.name}\n"
        out_string += f"\tparent {self.parent_name}\n"
        out_string += f"\tposition {self.position[0]:0.4f} {self.position[1]:0.4f} {self.position[2]:0.4f}\n"
        out_string += f"\ttechnical {1 if self.is_technical else 0}\n"
        out_string += f"\tanatomical {1 if self.is_anatomical else 0}\n"
        out_string += "endmarker\n"
        return out_string

    def __add__(self, other: np.ndarray | tuple):
        if isinstance(other, tuple):
            other = np.array(other)

        if isinstance(other, np.ndarray):
            return Marker(name=self.name, parent_name=self.parent_name, position=self.position + other)
        elif isinstance(other, Marker):
            return Marker(name=self.name, parent_name=self.parent_name, position=self.position + other.position)
        else:
            raise NotImplementedError(f"The addition for {type(other)} is not implemented")

    def __sub__(self, other):
        if isinstance(other, tuple):
            other = np.array(other)

        if isinstance(other, np.ndarray):
            return Marker(name=self.name, parent_name=self.parent_name, position=self.position - other)
        elif isinstance(other, Marker):
            return Marker(name=self.name, parent_name=self.parent_name, position=self.position - other.position)
        else:
            raise NotImplementedError(f"The subtraction for {type(other)} is not implemented")


class Axis:
    class Name(Enum):
        X = 0
        Y = 1
        Z = 2

    def __init__(self, name: Name, start: Marker, end: Marker):
        """
        Parameters
        ----------
        name:
            The AxisName of the Axis
        start:
            The initial Marker
        """
        self.name = name
        self.start_point = start
        self.end_point = end

    def axis(self) -> np.ndarray:
        """
        Compute the axis from actual data
        """

        if not ezc3d_found:
            raise RuntimeError("Ezc3d must be install to use the 'get_axis_from_data' constructor")

        start = self.start_point.position
        end = self.end_point.position
        return end - start


class RT:
    def __init__(self, rt: np.ndarray = np.identity(4), parent_rt: "RT" = None, is_rt_local: bool = False):
        """
        Parameters
        ----------
        rt
            The rt of the RT
        parent_rt
            The rt of the parent (is used when printing the model so RT is in parent's local reference frame
        is_rt_local
            If the rt is already in local reference frame
        """

        self.rt = rt
        self.parent_rt = parent_rt
        self.is_rt_local = is_rt_local

    @staticmethod
    def from_markers(origin: Marker, axes: tuple[Axis, Axis, Axis.Name], parent_rt: "RT" = None) -> "RT":
        """
        Parameters
        ----------
        origin
            The marker at the origin of the RT
        axes
            The axes that defines the RT, the AxisName is the axis to keep while constructing the RT
        parent_rt
            The rt of the parent (is used when printing the model so RT is in parent's local reference frame
        """

        # Find the two adjacent axes and reorder accordingly (assuming right-hand RT)
        first_axis = axes[0]
        second_axis = axes[1]
        axis_name_to_keep = axes[2]
        if first_axis.name == second_axis.name:
            raise ValueError("The two axes cannot be the same axis")

        if first_axis.name == Axis.Name.X:
            third_axis_name = Axis.Name.Y if second_axis.name == Axis.Name.Z else Axis.Name.Z
            if second_axis.name == Axis.Name.Z:
                first_axis, second_axis = second_axis, first_axis
        elif first_axis.name == Axis.Name.Y:
            third_axis_name = Axis.Name.Z if second_axis.name == Axis.Name.X else Axis.Name.X
            if second_axis.name == Axis.Name.X:
                first_axis, second_axis = second_axis, first_axis
        elif first_axis.name == Axis.Name.Z:
            third_axis_name = Axis.Name.X if second_axis.name == Axis.Name.Y else Axis.Name.Y
            if second_axis.name == Axis.Name.Y:
                first_axis, second_axis = second_axis, first_axis
        else:
            raise ValueError("first_axis should be an X, Y or Z axis")

        # Compute the third axis and recompute one of the previous two
        first_axis_vector = first_axis.axis()
        second_axis_vector = second_axis.axis()
        third_axis_vector = np.cross(first_axis_vector, second_axis_vector)
        if axis_name_to_keep == first_axis.name:
            second_axis_vector = np.cross(third_axis_vector, first_axis_vector)
        elif axis_name_to_keep == second_axis.name:
            first_axis_vector = np.cross(second_axis_vector, third_axis_vector)
        else:
            raise ValueError("Name of axis to keep should be one of the two axes")

        # Dispatch the result into a matrix
        rt = np.zeros((4, 4))
        rt[:3, first_axis.name.value] = first_axis_vector / np.linalg.norm(first_axis_vector)
        rt[:3, second_axis.name.value] = second_axis_vector / np.linalg.norm(second_axis_vector)
        rt[:3, third_axis_name.value] = third_axis_vector / np.linalg.norm(third_axis_vector)
        rt[:3, 3] = origin.position
        rt[3, 3] = 1

        return RT(rt=rt, parent_rt=parent_rt)

    @staticmethod
    def from_euler_and_translation(
        angles: tuple[float | int, ...],
        angle_sequence: str,
        translations: tuple[float | int, float | int, float | int],
        parent_rt: "RT" = None,
    ) -> "RT":
        """
        Construct a RT from angles and translations

        Parameters
        ----------
        angles
            The actual angles
        angle_sequence
            The angle sequence of the angles
        translations
            The XYZ translations
        parent_rt
            The rt of the parent (is used when printing the model so RT is in parent's local reference frame
        """
        matrix = {
            "x": lambda x: np.array(((1, 0, 0), (0, np.cos(x), -np.sin(x)), (0, np.sin(x), np.cos(x)))),
            "y": lambda y: np.array(((np.cos(y), 0, np.sin(y)), (0, 1, 0), (-np.sin(y), 0, np.cos(y)))),
            "z": lambda z: np.array(((np.cos(z), -np.sin(z), 0), (np.sin(z), np.cos(z), 0), (0, 0, 1))),
        }
        rt = np.identity(4)
        for angle, axis in zip(angles, angle_sequence):
            rt[:3, :3] = rt[:3, :3] @ matrix[axis](angle)
        rt[:3, 3] = translations
        return RT(rt=rt, parent_rt=parent_rt, is_rt_local=True)

    def copy(self):
        return RT(rt=copy(self.rt), parent_rt=self.parent_rt)

    def __str__(self):
        if self.is_rt_local:
            rt = self.rt
        else:
            rt = self.parent_rt.transpose @ self.rt if self.parent_rt else np.identity(4)

        tx = rt[0, 3]
        ty = rt[1, 3]
        tz = rt[2, 3]

        rx = np.arctan2(-rt[1, 2], rt[2, 2])
        ry = np.arcsin(rt[0, 2])
        rz = np.arctan2(-rt[0, 1], rt[0, 0])

        return f"{rx:0.3f} {ry:0.3f} {rz:0.3f} xyz {tx:0.3f} {ty:0.3f} {tz:0.3f}"

    def __matmul__(self, other):
        return self.rt @ other

    @property
    def transpose(self):
        out = self.copy()
        out.rt = out.rt.T
        out.rt[:3, 3] = -out.rt[:3, :3] @ out.rt[3, :3]
        out.rt[3, :3] = 0
        return out


class Segment:
    def __init__(
        self,
        name,
        parent_name: str = "",
        rt: RT = None,
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
        self.rt = rt
        self.mass = mass
        self.center_of_mass = center_of_mass if center_of_mass is not None else (0, 0, 0)
        self.inertia_xxyyzz = inertia_xxyyzz if inertia_xxyyzz is not None else (0, 0, 0)
        self.mesh = mesh

    def add_marker(self, marker: Marker):
        self.markers.append(marker)

    def __str__(self):
        # Define the print function so it automatically format things in the file properly<
        out_string = f"segment {self.name}\n"
        if self.parent_name:
            out_string += f"\tparent {self.parent_name}\n"
        if self.rt:
            out_string += f"\tRT {self.rt}\n"
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


class KinematicChain:
    def __init__(self, segments: tuple[Segment, ...]):
        self.segments = segments

    def __str__(self):
        out_string = "version 4\n\n"
        for segment in self.segments:
            out_string += str(segment)
            out_string += "\n\n\n"  # Give some space between segments
        return out_string

    def write(self, file_path: str):
        # Method to write the current KinematicChain to a file
        with open(file_path, "w") as file:
            file.write(str(self))


class DeLeva:
    # The DeLeva class is based on DeLeva (1996) "Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters"
    class Param:
        def __init__(
            self,
            marker_names: tuple[str, ...],  # The name of the markers medial/lateral
            mass: float | int,  # Percentage of the total body mass
            center_of_mass: float | int,
            # Position of the center of mass as a percentage of the distance from medial to distal
            radii: tuple[float | int, float | int, float | int],
            # [Sagittal, Transverse, Longitudinal] radii of giration
        ):
            self.marker_names = marker_names
            self.mass = mass
            self.center_of_mass = center_of_mass
            self.radii = radii

    def __init__(self, sex: str, mass: float | int, model: biorbd.Model):
        self.sex = sex  # The sex of the subject
        self.mass = mass  # The mass of the subject
        self.model = model  # The biorbd model. This is to compute lengths

        # Produce some easy to access variables
        self.q_zero = np.zeros((model.nbQ()))
        self.marker_names = [name.to_string() for name in model.markerNames()]

        # This is the actual copy of the DeLeva table
        self.table = {
            "male": {
                "HEAD": DeLeva.Param(
                    marker_names=("TOP_HEAD", "SHOULDER"),
                    mass=0.0694,
                    center_of_mass=0.5002,
                    radii=(0.303, 0.315, 0.261),
                ),
                "TRUNK": DeLeva.Param(
                    marker_names=("SHOULDER", "PELVIS"), mass=0.4346, center_of_mass=0.5138, radii=(0.328, 0.306, 0.169)
                ),
                "UPPER_ARM": DeLeva.Param(
                    marker_names=("SHOULDER", "ELBOW"),
                    mass=0.0271 * 2,
                    center_of_mass=0.5772,
                    radii=(0.285, 0.269, 0.158),
                ),
                "LOWER_ARM": DeLeva.Param(
                    marker_names=("ELBOW", "WRIST"), mass=0.0162 * 2, center_of_mass=0.4574, radii=(0.276, 0.265, 0.121)
                ),
                "HAND": DeLeva.Param(
                    marker_names=("WRIST", "FINGER"),
                    mass=0.0061 * 2,
                    center_of_mass=0.7900,
                    radii=(0.628, 0.513, 0.401),
                ),
                "THIGH": DeLeva.Param(
                    marker_names=("PELVIS", "KNEE"), mass=0.1416 * 2, center_of_mass=0.4095, radii=(0.329, 0.329, 0.149)
                ),
                "SHANK": DeLeva.Param(
                    marker_names=("KNEE", "ANKLE"), mass=0.0433 * 2, center_of_mass=0.4459, radii=(0.255, 0.249, 0.103)
                ),
                "FOOT": DeLeva.Param(
                    marker_names=("ANKLE", "TOE"), mass=0.0137 * 2, center_of_mass=0.4415, radii=(0.257, 0.245, 0.124)
                ),
            },
            "female": {
                "HEAD": DeLeva.Param(
                    marker_names=("TOP_HEAD", "SHOULDER"),
                    mass=0.0669,
                    center_of_mass=0.4841,
                    radii=(0.271, 0.295, 0.261),
                ),
                "TRUNK": DeLeva.Param(
                    marker_names=("SHOULDER", "PELVIS"), mass=0.4257, center_of_mass=0.4964, radii=(0.307, 0.292, 0.147)
                ),
                "UPPER_ARM": DeLeva.Param(
                    marker_names=("SHOULDER", "ELBOW"),
                    mass=0.0255 * 2,
                    center_of_mass=0.5754,
                    radii=(0.278, 0.260, 0.148),
                ),
                "LOWER_ARM": DeLeva.Param(
                    marker_names=("ELBOW", "WRIST"), mass=0.0138 * 2, center_of_mass=0.4559, radii=(0.261, 0.257, 0.094)
                ),
                "HAND": DeLeva.Param(
                    marker_names=("WRIST", "FINGER"),
                    mass=0.0056 * 2,
                    center_of_mass=0.7474,
                    radii=(0.531, 0.454, 0.335),
                ),
                "THIGH": DeLeva.Param(
                    marker_names=("PELVIS", "KNEE"), mass=0.1478 * 2, center_of_mass=0.3612, radii=(0.369, 0.364, 0.162)
                ),
                "SHANK": DeLeva.Param(
                    marker_names=("KNEE", "ANKLE"), mass=0.0481 * 2, center_of_mass=0.4416, radii=(0.271, 0.267, 0.093)
                ),
                "FOOT": DeLeva.Param(
                    marker_names=("ANKLE", "TOE"), mass=0.0129 * 2, center_of_mass=0.4014, radii=(0.299, 0.279, 0.124)
                ),
            },
        }

    def segment_mass(self, segment: Segment):
        return self.table[self.sex][segment].mass * self.mass

    def segment_length(self, segment: Segment):
        table = self.table[self.sex][segment]

        # Find the position of the markers when the model is in resting position
        marker_positions = np.array([marker.to_array() for marker in self.model.markers(self.q_zero)]).transpose()

        # Find the index of the markers required to compute the length of the segment
        idx_proximal = self.marker_names.index(table.marker_names[0])
        idx_distal = self.marker_names.index(table.marker_names[1])

        # Compute the Euclidian distance between the two positions
        return np.linalg.norm(marker_positions[:, idx_distal] - marker_positions[:, idx_proximal])

    def segment_center_of_mass(self, segment: Segment, inverse_proximal: bool = False):
        # This method will compute the length of the required segment based on the biorbd model and required markers
        # If inverse_proximal is set to True, then the value is returned from the distal position
        table = self.table[self.sex][segment]

        # Find the position of the markers when the model is in resting position
        marker_positions = np.array([marker.to_array() for marker in self.model.markers(self.q_zero)]).transpose()

        # Find the index of the markers required to compute the length of the segment
        idx_proximal = self.marker_names.index(table.marker_names[0])
        idx_distal = self.marker_names.index(table.marker_names[1])

        # Compute the position of the center of mass
        if inverse_proximal:
            center_of_mass = (1 - table.center_of_mass) * (
                marker_positions[:, idx_proximal] - marker_positions[:, idx_distal]
            )
        else:
            center_of_mass = table.center_of_mass * (
                marker_positions[:, idx_distal] - marker_positions[:, idx_proximal]
            )
        return tuple(center_of_mass)  # convert the result to a Tuple which is good practise

    def segment_moment_of_inertia(self, segment: Segment):
        mass = self.segment_mass(segment)
        length = self.segment_length(segment)
        radii = self.table[self.sex][segment].radii

        return mass * (length * radii[0]) ** 2, mass * (length * radii[1]) ** 2, mass * (length * radii[2]) ** 2


class MarkerGeneric:
    def __init__(self, name: str, from_markers: str | tuple[str, ...], parent_name: str, is_technical: bool = True, is_anatomical: bool = False):
        """
        This is a pre-constructor for the Marker class. It allows to create a generic model by marker names

        Parameters
        ----------
        name:
            The name of the new marker
        from_markers:
            The name of the markers in the data
        parent_name:
            The name of the parent the marker is attached to
        is_technical
            If the marker should be flaged as a technical marker
        is_anatomical
            If the marker should be flaged as an anatomical marker
        """
        self.name = name
        self.from_markers = from_markers
        self.parent_name = parent_name
        self.is_technical = is_technical
        self.is_anatomical = is_anatomical

    def to_marker(self, c3d: ezc3d.c3d, parent_rt: RT = None) -> Marker:
        return Marker.from_data(c3d, self.name, self.from_markers, self.parent_name, parent_rt, is_technical=self.is_technical, is_anatomical=self.is_anatomical)


class AxisGeneric:
    def __init__(self, name: Axis.Name, start: MarkerGeneric, end: MarkerGeneric):
        """
        Parameters
        ----------
        name:
            The AxisName of the Axis
        start:
            The initial Marker
        """
        self.name = name
        self.start_point = start
        self.end_point = end

    def to_axis(self, c3d: ezc3d.c3d, parent_rt: RT = None) -> Axis:
        """
        Compute the axis from actual data
        Parameters
        ----------
        c3d:
            The ezc3d file containing the data
        parent_rt:
            The transformation from global to local
        """

        if not ezc3d_found:
            raise RuntimeError("Ezc3d must be install to use the 'get_axis_from_data' constructor")

        start = self.start_point.to_marker(c3d, parent_rt)
        end = self.end_point.to_marker(c3d, parent_rt)
        return Axis(self.name, start, end)


class RTGeneric:
    def __init__(self, origin: MarkerGeneric, axes: tuple[AxisGeneric, AxisGeneric, Axis.Name]):
        """
        Parameters
        ----------
        origin
            The origin of the RT
        axes
            The first (axes[0]) and second (axes[1]) axes of the RT with the name of the recomputed axis (axis[2])
        """
        self.origin = origin
        self.axes = axes

    def to_rt(self, c3d: ezc3d.c3d, parent_rt: RT) -> RT:
        """
        Collapse the generic RT to a RT with value based on the model and the c3d data

        Parameters
        ----------
        c3d
            The c3d data that provides the values
        parent_rt
            The RT of the parent to compute the local transformation
        Returns
        -------
        The collapsed RT
        """
        origin = self.origin.to_marker(c3d)
        axes = (self.axes[0].to_axis(c3d), self.axes[1].to_axis(c3d), self.axes[2])

        return RT.from_markers(origin, axes, parent_rt)


class SegmentGeneric:
    def __init__(
        self,
        name,
        parent_name: str = "",
        translations: str = "",
        rotations: str = "",
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
        self.rt = None

    def add_marker(self, marker: MarkerGeneric):
        """
        Add a new marker to the segment

        Parameters
        ----------
        marker
            The marker to add
        """
        self.markers.append(marker)

    def set_rt(
        self,
        origin_from_markers: str | tuple[str, ...],
        first_axis_name: Axis.Name,
        first_axis_markers: tuple[str | tuple[str, ...], str | tuple[str, ...]],
        second_axis_name: Axis.Name,
        second_axis_markers: tuple[str | tuple[str, ...], str | tuple[str, ...]],
        axis_to_keep: Axis.Name,
    ):
        """
        Define the rt of the segment.

        Parameters
        ----------
        origin_from_markers
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
            origin=MarkerGeneric(name="", from_markers=origin_from_markers, parent_name=""),
            axes=(first_axis_tp, second_axis_tp, axis_to_keep),
        )


class KinematicModelGeneric:
    def __init__(self, bio_sym_path: str = None):
        self.segments = {}
        if bio_sym_path is None:
            return
        raise NotImplementedError("bioMod files are not readable yet")

    def add_segment(
        self,
        name: str,
        parent_name: str = "",
        translations: str = "",
        rotations: str = "",
    ):
        """
        Add a new segment to the model

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
        self.segments[name] = SegmentGeneric(
            name=name, parent_name=parent_name, translations=translations, rotations=rotations
        )

    def set_rt(
        self,
        segment_name,
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
        segment_name
            The segment name to set the rt matrix
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
        self.segments[segment_name].set_rt(
            origin_from_markers=origin_markers,
            first_axis_name=first_axis_name,
            first_axis_markers=first_axis_markers,
            second_axis_name=second_axis_name,
            second_axis_markers=second_axis_markers,
            axis_to_keep=axis_to_keep,
        )

    def add_marker(
        self,
        segment: str,
        name: str,
        from_markers: str | tuple[str, ...] = None,
        is_technical: bool = True,
        is_anatomical: bool = False,
    ):
        """
        Add a new marker to the specified segment
        Parameters
        ----------
        segment
            The name of the segment to attach the marker to
        name
            The name of the marker. It must be unique accross the model
        from_markers
            The name of the markers to create the marker from. It is used to create virtual marker from
            combination of other markers. If it is empty, the marker is normal
        is_technical
            If the marker should be flaged as a technical marker
        is_anatomical
            If the marker should be flaged as an anatomical marker
        """
        if from_markers is None:
            from_markers = name
        self.segments[segment].add_marker(MarkerGeneric(name=name, from_markers=from_markers, parent_name=segment, is_technical=is_technical, is_anatomical=is_anatomical))

    def generate_personalized(self, c3d: ezc3d, save_path: str):
        """
        Collapse the model to an actual personalized kinematic chain based on the model and the c3d file

        Parameters
        ----------
        c3d
            The c3d file to create the model from
        save_path
            The path to save the bioMod
        """
        segments = []
        for name in self.segments:
            s = self.segments[name]
            parent_index = [segment.name for segment in segments].index(s.parent_name) if s.parent_name else None
            if s.rt is None:
                rt = RT()
            else:
                rt = s.rt.to_rt(c3d, segments[parent_index].rt if parent_index is not None else None)
            segments.append(
                Segment(
                    name=s.name,
                    parent_name=s.parent_name,
                    rt=rt,
                    translations=s.translations,
                    rotations=s.rotations,
                )
            )

            for marker in s.markers:
                segments[-1].add_marker(marker.to_marker(c3d, rt))

        model = KinematicChain(tuple(segments))
        model.write(save_path)
