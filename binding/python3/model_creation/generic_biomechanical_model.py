from typing import Callable

from .kinematic_chain import KinematicChain
from .marker_generic import MarkerGeneric
from .protocols import Data, GenericDynamicModel
from .segment import Segment
from .segment_coordinate_system_generic import SegmentCoordinateSystemGeneric
from .segment_generic import SegmentGeneric
from .rototranslation import RT


class GenericBiomechanicalModel:
    def __init__(self, bio_sym_path: str = None):
        self.segments = {}
        self._dynamic_model = None
        if bio_sym_path is None:
            return
        raise NotImplementedError("bioMod files are not readable yet")

    @property
    def dynamic_model(self) -> GenericDynamicModel:
        """
        Get the dynamic model attached to the kinematic model
        """
        return self._dynamic_model

    @dynamic_model.setter
    def dynamic_model(self, dynamic_model: GenericDynamicModel) -> None:
        """
        Attach a dynamic model to the kinematic model. The name of the segments must exactly match the name of the
        segments of the kinematic model, otherwise a ValueError is raised

        Parameters
        ----------
        dynamic_model
            The model to attach
        """

        # Perform a check that all the names in the dynamic model appear in the kinematic model
        segments_in_dynamics = dynamic_model.segment_names
        for name in segments_in_dynamics:
            if name not in self.segments:
                raise ValueError(f'The segment {name} is defined in the dynamic model, but not in the kinematic model')

        # Perform the same, but the other way around
        for name in self.segments:
            if name not in segments_in_dynamics:
                raise ValueError(f'The segment {name} is defined in the kinematic model, but not in the dynamic model')

        self._dynamic_model = dynamic_model

    def add_segment(
        self,
        name: str,
        parent_name: str = "",
        translations: str = "",
        rotations: str = "",
        segment_coordinate_system: SegmentCoordinateSystemGeneric = None,
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
        """
        self.segments[name] = SegmentGeneric(
            name=name,
            parent_name=parent_name,
            translations=translations,
            rotations=rotations,
            segment_coordinate_system=segment_coordinate_system,
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
            MarkerGeneric(
                name=name,
                function=function if function is not None else lambda m: m[name],
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

        segments = []
        for name in self.segments:
            s = self.segments[name]
            parent_index = [segment.name for segment in segments].index(s.parent_name) if s.parent_name else None
            if s.segment_coordinate_system is None:
                scs = RT()
            else:
                scs = s.segment_coordinate_system.to_rt(
                    data, segments[parent_index].segment_coordinate_system if parent_index is not None else None
                )
            segments.append(
                Segment(
                    name=s.name,
                    parent_name=s.parent_name,
                    segment_coordinate_system=scs,
                    translations=s.translations,
                    rotations=s.rotations,
                )
            )

            for marker in s.markers:
                segments[-1].add_marker(marker.to_marker(data, scs))

        model = KinematicChain(tuple(segments))
        model.write(save_path)
