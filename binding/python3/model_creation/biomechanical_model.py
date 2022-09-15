from typing import Callable

from .inertia_parameters import InertiaParameters
from .kinematic_chain import KinematicChain
from .marker import Marker
from .mesh import Mesh
from .protocols import Data
from .segment_real import SegmentReal
from .segment_coordinate_system import SegmentCoordinateSystem
from .segment import Segment
from .segment_coordinate_system_real import SegmentCoordinateSystemReal


class BiomechanicalModel:
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
        segment_coordinate_system: SegmentCoordinateSystem = None,
        inertia_parameters: InertiaParameters = None,
        mesh: Mesh = None,
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
        segment_coordinate_system
            The coordinate system of the segment
        inertia_parameters
            The inertia parameters of the segment
        mesh
            The mesh points of the segment
        """
        self.segments[name] = Segment(
            name=name,
            parent_name=parent_name,
            translations=translations,
            rotations=rotations,
            segment_coordinate_system=segment_coordinate_system,
            inertia_parameters=inertia_parameters,
            mesh=mesh,
        )

    def add_marker(
        self,
        segment: str,
        name: str,
        function: Callable = None,
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
            The name of the marker. It must be unique across the model
        function
            The function (f(m) -> np.ndarray, where m is a dict of markers (XYZ1 x time)) that defines the marker
        is_technical
            If the marker should be flagged as a technical marker
        is_anatomical
            If the marker should be flagged as an anatomical marker
        """

        self.segments[segment].add_marker(
            Marker(
                name=name,
                function=function if function is not None else name,
                parent_name=segment,
                is_technical=is_technical,
                is_anatomical=is_anatomical,
            )
        )

    def write(self, save_path: str, data: Data):
        """
        Collapse the model to an actual personalized kinematic chain based on the model and the data file (usually
        a static trial)

        Parameters
        ----------
        save_path
            The path to save the bioMod
        data
            The data to collapse the model from
        """

        model = KinematicChain()
        for name in self.segments:
            s = self.segments[name]

            scs = SegmentCoordinateSystemReal()
            if s.segment_coordinate_system is not None:
                scs = s.segment_coordinate_system.to_scs(
                    data,
                    model,
                    model.segments[s.parent_name].segment_coordinate_system if s.parent_name else None,
                )

            inertia_parameters = None
            if s.inertia_parameters is not None:
                inertia_parameters = s.inertia_parameters.to_real(data, model, scs)

            mesh = None
            if s.mesh is not None:
                mesh = s.mesh.to_mesh(data, model, scs)

            model.add_segment(
                SegmentReal(
                    name=s.name,
                    parent_name=s.parent_name,
                    segment_coordinate_system=scs,
                    translations=s.translations,
                    rotations=s.rotations,
                    inertia_parameters=inertia_parameters,
                    mesh=mesh,
                )
            )

            for marker in s.markers:
                model.segments[name].add_marker(marker.to_marker(data, model, scs))

        model.write(save_path)
