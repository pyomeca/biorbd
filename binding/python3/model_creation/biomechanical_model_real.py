import os.path
import numpy as np

from biorbd import Model

from build_casadi.binding.python3.biorbd import MuscleGroup


class BiomechanicalModelReal:
    def __init__(self, biomod_path: str = None):
        from .segment_real import SegmentReal  # Imported here to prevent from circular imports
        from .muscle_group import MuscleGroup
        from .muscle_real import MuscleReal
        from .via_point_real import ViaPointReal

        self.segments: dict[str:SegmentReal, ...] = {}
        # From Pythom 3.7 the insertion order in a dict is preserved. This is important because when writing a new
        # .bioMod file, the order of the segment matters
        self.muscle_groups: dict[str:MuscleGroup, ...] = {}
        self.muscles: dict[str:MuscleReal, ...] = {}
        self.via_points: dict[str:ViaPointReal, ...] = {}

        if biomod_path is not None:
            if os.path.exists(biomod_path):
                print(f"Reading from file {os.path.abspath(biomod_path)}")
                self.read(biomod_path)
            else:
                raise RuntimeError(f"The file {os.path.abspath(biomod_path)} does not exist, therefore it cannot be read.")


    def read(self, biomod_path: str):
        """
        This method allows to read a bioMod file to fasten the creation of a BiomechanicalModel.

        Parameters
        ----------
        biomod_path: str
            The path to the bioMod file to read
        """
        from .segment_real import SegmentReal  # Imported here to prevent from circular imports
        from .muscle_group import MuscleGroup
        from .muscle_real import MuscleReal
        from .via_point_real import ViaPointReal
        from .inertia_parameters_real import InertiaParametersReal
        from .translations import Translations
        from .rotations import Rotations
        from .range_of_motion import RangeOfMotion, Ranges
        from .segment_coordinate_system_real import SegmentCoordinateSystemReal
        from .marker_real import MarkerReal
        from .contact_real import ContactReal


        def get_segment_name(segment):
            return segment.name().to_string()

        def get_segment_parent_name(segment):
            return segment.parent().to_string()

        def get_segment_translation(segment):
            trans_name = segment.seqT().to_string()
            if trans_name == '':
                return Translations.NONE
            else:
                return Translations(trans_name)

        def get_segment_rotation(segment):
            rot_name = segment.seqR().to_string()
            if rot_name == '':
                return Rotations.NONE
            else:
                return Rotations(rot_name)

        def get_segment_q_range(segment):
            if len(segment.QRanges()) == 0:
                return None
            else:
                segment_ranges = segment.QRanges()
                q_min = []
                q_max = []
                for i_range, range in enumerate(segment_ranges):
                    q_min.append(range.min())
                    q_max.append(range.max())
                return RangeOfMotion(range_type=Ranges.Q, min_bound=q_min, max_bound=q_max)

        def get_segment_qdot_range(segment):
            if len(segment.QdotRanges()) == 0:
                return None
            else:
                segment_ranges = segment.QdotRanges()
                qdot_min = []
                qdot_max = []
                for i_range, range in enumerate(segment_ranges):
                    qdot_min.append(range.min())
                    qdot_max.append(range.max())
                return RangeOfMotion(range_type=Ranges.Qdot, min_bound=qdot_min, max_bound=qdot_max)

        def get_segment_coordinate_system(segment):
            return SegmentCoordinateSystemReal(scs=segment.localJCS().to_array())

        def get_segment_inertia_parameters(segment):
            mass = segment.characteristics().mass()
            center_of_mass = segment.characteristics().CoM().to_array()
            inertia = np.diag(segment.characteristics().inertia().to_array())
            inertial_parameters = InertiaParametersReal(mass=mass,
                                                    center_of_mass=center_of_mass.reshape(-1, 1),
                                                    inertia=inertia)
            return inertial_parameters

        def get_contact_axis(contact):
            # TODO: Charbie -> to implement when I know where it is accessible --'
            # "copy" from get_segment_translation
            pass


        model_to_load = Model(biomod_path)

        # Fill segments' information
        for i_segment, segment in enumerate(model_to_load.segments()):
            # TODO: add meshes and mesh_files, @pariterre: I cannot find how to extract meshes :(
            name = get_segment_name(segment)
            self.segments[name] = SegmentReal(
                name=name,
                parent_name=get_segment_parent_name(segment),
                translations=get_segment_translation(segment),
                rotations=get_segment_rotation(segment),
                q_ranges=get_segment_q_range(segment),
                qdot_ranges=get_segment_qdot_range(segment),
                segment_coordinate_system=get_segment_coordinate_system(segment),
                inertia_parameters=get_segment_inertia_parameters(segment),
            )
        # Fill markers' information
        marker_names = [m.to_string() for m in model_to_load.markerNames()]
        segment_names = list(self.segments.keys())
        parent_ids = [str(m.parentId()) for m in model_to_load.markers()]
        segment_ids = [str(s.id()) for s in model_to_load.segments()]
        for i_marker, marker in enumerate(model_to_load.markers()):
            parent_idx = segment_ids.index(parent_ids[i_marker])
            parent_name = segment_names[parent_idx]
            self.segments[parent_name].add_marker(MarkerReal(name=marker_names[i_marker],
                                                            parent_name=parent_name,
                                                            position=marker.to_array()))
        # Fill contacts' information
        contact_names = [c.to_string() for c in model_to_load.contactNames()]
        segment_names = list(self.segments.keys())
        parent_ids = [str(c.parentId()) for c in model_to_load.rigidContacts()]
        segment_ids = [str(s.id()) for s in model_to_load.segments()]
        for i_contact, contact in enumerate(model_to_load.rigidContacts()):
            parent_idx = segment_ids.index(parent_ids[i_contact])
            parent_name = segment_names[parent_idx]
            self.segments[parent_name].add_contact(ContactReal(name=contact_names[i_contact],
                                                            parent_name=parent_name,
                                                            position=contact.to_array()),
                                                            axis=get_contact_axis(contact))

        # Fill muscle groups' information
        for i_muscle_group, muscle_group in enumerate(model_to_load.muscleGroups()):
            name = muscle_group.name().to_string()
            self.muscle_groups[name] = MuscleGroup(name=name,
                                                    origin_parent_name=muscle_group.origin().to_string(),
                                                    insertion_parent_name=muscle_group.insertion().to_string())
        # Fill via points' information
        # TODO: Charbie -> No idea where this is stored
        # for i_via_point, via_point in enumerate(model_to_load.viaPoints()):

        # Fill muscles' information
        for i_muscle, muscle in enumerate(model_to_load.muscles()):
            name = muscle.name().to_string()
            self.muscle_groups[name] = MuscleReal(name=name,
                                                  muscle_type=muscle.type(),  # TODO: This returns an int
                                                  state_type=muscle.stateType(),  # TODO: Does not exist
                                                  muscle_group=muscle.muscleGroup(),  # TODO: Does not exist
                                                  origin_position=muscle.position().originInLocal().to_array(),
                                                  insertion_position=muscle.position().insertionInLocal().to_array(),
                                                  optimal_length=muscle.characteristics().optimalLength(),
                                                  maximal_force=muscle.characteristics().forceIsoMax(),
                                                  tendon_slack_length=muscle.characteristics().tendonSlackLength(),
                                                  pennation_angle=muscle.characteristics().pennationAngle(),
                                                  maximal_excitation=muscle.maximalExcitation(),  # TOD: Does not exist
                                                  via_points=muscle.viaPoints())  # TODO: Does not exist


    def __str__(self):
        out_string = "version 4\n\n"

        out_string += "// --------------------------------------------------------------\n"
        out_string += "// SEGMENTS\n"
        out_string += "// --------------------------------------------------------------\n\n"
        for name in self.segments:
            out_string += str(self.segments[name])
            out_string += "\n\n\n"  # Give some space between segments

        out_string += "// --------------------------------------------------------------\n"
        out_string += "// MUSCLE GROUPS\n"
        out_string += "// --------------------------------------------------------------\n\n"
        for name in self.muscle_groups:
            out_string += str(self.muscle_groups[name])
            out_string += "\n"
        out_string += "\n\n\n"  # Give some space after muscle groups

        out_string += "// --------------------------------------------------------------\n"
        out_string += "// MUSCLES\n"
        out_string += "// --------------------------------------------------------------\n\n"
        for name in self.muscles:
            out_string += str(self.muscles[name])
            out_string += "\n\n\n"  # Give some space between muscles

        out_string += "// --------------------------------------------------------------\n"
        out_string += "// MUSCLES VIA POINTS\n"
        out_string += "// --------------------------------------------------------------\n\n"
        for name in self.via_points:
            out_string += str(self.via_points[name])
            out_string += "\n\n\n"  # Give some space between via points

        return out_string

    def write(self, file_path: str):
        # Method to write the current KinematicChain to a file
        with open(file_path, "w") as file:
            file.write(str(self))
