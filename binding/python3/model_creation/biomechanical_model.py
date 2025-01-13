from .protocols import Data
from .segment_real import SegmentReal
from .muscle_group import MuscleGroup
from .muscle_real import MuscleReal
from .segment_coordinate_system_real import SegmentCoordinateSystemReal
from .biomechanical_model_real import BiomechanicalModelReal


class BiomechanicalModel:
    def __init__(self, bio_sym_path: str = None):
        self.segments = {}
        self.muscle_groups = {}
        self.muscles = {}

        if bio_sym_path is None:
            return
        raise NotImplementedError("bioMod files are not readable yet")

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

            model.segments[s.name] = SegmentReal(
                name=s.name,
                parent_name=s.parent_name,
                segment_coordinate_system=scs,
                translations=s.translations,
                rotations=s.rotations,
                q_ranges=s.q_ranges,
                qdot_ranges=s.qdot_ranges,
                inertia_parameters=inertia_parameters,
                mesh=mesh,
            )

            for marker in s.markers:
                model.segments[name].add_marker(marker.to_marker(data, model, scs))

            for contact in s.contacts:
                model.segments[name].add_contact(contact.to_contact(data))

        for name in self.muscle_groups:
            mg = self.muscle_groups[name]

            model.muscle_groups[mg.name] = MuscleGroup(
                name=mg.name,
                origin_parent_name=mg.origin_parent_name,
                insertion_parent_name=mg.insertion_parent_name,
            )

        for name in self.muscles:
            m = self.muscles[name]

            if m.muscle_group not in model.muscle_groups:
                raise RuntimeError(f"Please create the muscle group {m.muscle_group} before putting the muscle {m.name} in it.")

            model.muscles[m.name] = MuscleReal(
                name=m.name,
                type=m.type,
                state_type=m.state_type,
                muscle_group=m.muscle_group,
                origin_position=m.origin_position,
                insertion_position=m.insertion_position,
                optimal_length=m.optimal_length,
                maximal_force=m.maximal_force,
                tendon_slack_length=m.tendon_slack_length,
                penation_angle=m.penation_angle)

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
