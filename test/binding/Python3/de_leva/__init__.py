from biorbd.model_creation import InertiaParameters
import numpy as np


def point_on_vector(coef: float, start: np.ndarray, end: np.ndarray) -> np.ndarray:
    return start + coef * (end - start)


class DeLevaTable:
    def __init__(self, total_mass: float, sex: str):
        """
        Implementation of the DeLeva table
        Parameters
        ----------
        total_mass
            The mass of the subject
        sex
            The sex ('male' or 'female') of the subject
        """

        self.sex = sex
        self.de_leva_table = {
            "male": {
                "HEAD": InertiaParameters(
                    mass=lambda m, bio: 0.0694 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.5002, start=m["TOP_HEAD"], end=m["SHOULDER"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=0.0694 * total_mass,
                        coef=(0.303, 0.315, 0.261),
                        start=m["TOP_HEAD"],
                        end=m["SHOULDER"],
                    ),
                ),
                "TRUNK": InertiaParameters(
                    mass=lambda m, bio: 0.4346 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.5138, start=m["SHOULDER"], end=m["PELVIS"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=0.4346 * total_mass,
                        coef=(0.328, 0.306, 0.169),
                        start=m["SHOULDER"],
                        end=m["PELVIS"],
                    ),
                ),
                "UPPER_ARM": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0271 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.5772, start=m["SHOULDER"], end=m["ELBOW"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0271 * total_mass,
                        coef=(0.285, 0.269, 0.158),
                        start=m["SHOULDER"],
                        end=m["ELBOW"],
                    ),
                ),
                "LOWER_ARM": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0162 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4574, start=m["ELBOW"], end=m["WRIST"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0162 * total_mass,
                        coef=(0.276, 0.265, 0.121),
                        start=m["ELBOW"],
                        end=m["WRIST"],
                    ),
                ),
                "HAND": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0061 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.7900, start=m["WRIST"], end=m["FINGER"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0061 * total_mass,
                        coef=(0.628, 0.513, 0.401),
                        start=m["WRIST"],
                        end=m["FINGER"],
                    ),
                ),
                "THIGH": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.1416 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4095, start=m["PELVIS"], end=m["KNEE"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.1416 * total_mass,
                        coef=(0.329, 0.329, 0.149),
                        start=m["PELVIS"],
                        end=m["KNEE"],
                    ),
                ),
                "SHANK": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0433 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4459, start=m["KNEE"], end=m["ANKLE"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0433 * total_mass,
                        coef=(0.255, 0.249, 0.103),
                        start=m["KNEE"],
                        end=m["ANKLE"],
                    ),
                ),
                "FOOT": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0137 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4415, start=m["ANKLE"], end=m["TOE"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0137 * total_mass,
                        coef=(0.257, 0.245, 0.124),
                        start=m["ANKLE"],
                        end=m["TOE"],
                    ),
                ),
            },
            "female": {
                "HEAD": InertiaParameters(
                    mass=lambda m, bio: 0.0669 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4841, start=m["TOP_HEAD"], end=m["SHOULDER"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=0.0669 * total_mass,
                        coef=(0.271, 0.295, 0.261),
                        start=m["TOP_HEAD"],
                        end=m["SHOULDER"],
                    ),
                ),
                "TRUNK": InertiaParameters(
                    mass=lambda m, bio: 0.4257 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4964, start=m["SHOULDER"], end=m["PELVIS"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=0.4257 * total_mass,
                        coef=(0.307, 0.292, 0.147),
                        start=m["SHOULDER"],
                        end=m["PELVIS"],
                    ),
                ),
                "UPPER_ARM": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0255 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.5754, start=m["SHOULDER"], end=m["ELBOW"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0255 * total_mass,
                        coef=(0.278, 0.260, 0.148),
                        start=m["SHOULDER"],
                        end=m["ELBOW"],
                    ),
                ),
                "LOWER_ARM": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0138 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4559, start=m["ELBOW"], end=m["WRIST"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0138 * total_mass,
                        coef=(0.261, 0.257, 0.094),
                        start=m["ELBOW"],
                        end=m["WRIST"],
                    ),
                ),
                "HAND": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0056 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.7474, start=m["WRIST"], end=m["FINGER"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0056 * total_mass,
                        coef=(0.531, 0.454, 0.335),
                        start=m["WRIST"],
                        end=m["FINGER"],
                    ),
                ),
                "THIGH": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.1478 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.3612, start=m["PELVIS"], end=m["KNEE"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.1478 * total_mass,
                        coef=(0.369, 0.364, 0.162),
                        start=m["PELVIS"],
                        end=m["KNEE"],
                    ),
                ),
                "SHANK": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0481 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4416, start=m["KNEE"], end=m["ANKLE"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0481 * total_mass,
                        coef=(0.271, 0.267, 0.093),
                        start=m["KNEE"],
                        end=m["ANKLE"],
                    ),
                ),
                "FOOT": InertiaParameters(
                    mass=lambda m, bio: 2 * 0.0129 * total_mass,
                    center_of_mass=lambda m, bio: point_on_vector(0.4014, start=m["ANKLE"], end=m["TOE"]),
                    inertia=lambda m, bio: InertiaParameters.radii_of_gyration_to_inertia(
                        mass=2 * 0.0129 * total_mass,
                        coef=(0.299, 0.279, 0.124),
                        start=m["ANKLE"],
                        end=m["TOE"],
                    ),
                ),
            },
        }

    def __getitem__(self, name: str):
        """
        The inertia paremeters for a particular segment
        Parameters
        ----------
        name
            The name of the segment
        """
        return self.de_leva_table[self.sex][name]
