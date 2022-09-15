from typing import Callable

from .inertia_parameters import InertiaParameters
from .biomechanical_model_real import BiomechanicalModelReal
from .marker import Marker
from .mesh import Mesh
from .protocols import Data
from .segment_real import SegmentReal
from .segment_coordinate_system import SegmentCoordinateSystem
from .segment import Segment
from .segment_coordinate_system_real import SegmentCoordinateSystemReal
from .biomechanical_model_real import BiomechanicalModelReal


class BiomechanicalModel:
    def __init__(self, bio_sym_path: str = None):
        self.segments = {}

        if bio_sym_path is None:
            return
        raise NotImplementedError("bioMod files are not readable yet")

    def add_segment(self, segment: Segment):
        """
        Add a new segment to the model

        Parameters
        ----------
        segment
            The segment to add
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

    def __getitem__(self, name: str):
        return self.segments[name]

    def __setitem__(self, name: str, segment: Segment):
        segment.name = name  # Make sure the name of the segment fits the internal one
        self.segments[name] = segment

    def to_real(self, data: Data) -> BiomechanicalModelReal:
        """
        Collapse the model to an actual personalized biomechanical model based on the generic model and the data
        file (usually a static trial)

        Parameters
        ----------
        data
            The data to collapse the model from
        """
        model = BiomechanicalModelReal()
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

            model[s.name] = SegmentReal(
                name=s.name,
                parent_name=s.parent_name,
                segment_coordinate_system=scs,
                translations=s.translations,
                rotations=s.rotations,
                inertia_parameters=inertia_parameters,
                mesh=mesh,
            )

            for marker in s.markers:
                model.segments[name].add_marker(marker.to_marker(data, model, scs))

        return model

    def write(self, save_path: str, data: Data):
        """
        Collapse the model to an actual personalized biomechanical model based on the generic model and the data
        file (usually a static trial)

        Parameters
        ----------
        save_path
            The path to save the bioMod
        data
            The data to collapse the model from
        """
        model = self.to_real(data)
        model.write(save_path)
