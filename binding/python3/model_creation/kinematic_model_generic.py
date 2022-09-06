from .kinematic_chain import KinematicChain
from .marker_generic import MarkerGeneric
from .protocols import Data
from .segment import Segment
from .segment_coordinate_system import SegmentCoordinateSystem
from .segment_generic import SegmentGeneric
from .rototranslation import RT


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
        segment_coordinate_system: SegmentCoordinateSystem = None,
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
        self.segments[segment].add_marker(
            MarkerGeneric(
                name=name,
                from_markers=from_markers,
                parent_name=segment,
                is_technical=is_technical,
                is_anatomical=is_anatomical,
            )
        )

    def generate_personalized(self, data: Data, save_path: str):
        """
        Collapse the model to an actual personalized kinematic chain based on the model and the data file

        Parameters
        ----------
        data
            The data to collapse the model from
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
                rt = s.rt.to_rt(data, segments[parent_index].rt if parent_index is not None else None)
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
                segments[-1].add_marker(marker.to_marker(data, rt))

        model = KinematicChain(tuple(segments))
        model.write(save_path)
