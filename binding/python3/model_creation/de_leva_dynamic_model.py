import biorbd
import numpy as np


class Param:
    """
    This is a convenient class to hold inertia parameter information of the DeLeva model below
    """
    def __init__(
        self,
        marker_names: tuple[str, ...],
        mass: float,
        center_of_mass: float,
        radii: tuple[float, float, float],
    ):
        """
        Parameters
        ----------
        marker_names
            The name of the markers that creates the vector to compute the length of the segment from medial to lateral
        mass
            The relative mass of the total body mass
        center_of_mass
            The position of the center of mass as a percentage of the distance from medial to distal
        radii
            The xx, yy, zz, (Sagittal, Transverse, Longitudinal) radii of giration
        """
        self.marker_names = marker_names
        self.mass = mass
        self.center_of_mass = center_of_mass
        self.radii = radii


class DeLevaDynamicModel:
    """
    This is the implementation from DeLeva (1996) "Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters"
    of the DynamicModel protocol of model_creation.
    Please note that this model is designed for a 2D model (that is, there is only one arm and one leg with their
    inertial parameters doubled).
    The kinematic model should be named as such (SEGMENT_NAME: from 'MEDIAL_MARKER' to 'LATERAL_MARKER'):
        'HEAD': from 'TOP_HEAD' to 'SHOULDER'
        'TRUNK': from 'SHOULDER' to 'PELVIS'
        'UPPER_ARM': from 'SHOULDER' to 'ELBOW'
        'LOWER_ARM': from 'ELBOW' to 'WRIST'
        'HAND': from 'WRIST' to 'FINGER'
        'THIGH': from 'PELVIS' to 'KNEE'
        'SHANK': from 'KNEE' to 'ANKLE'
        'FOOT': from 'ANKLE' to 'TOE'
    """

    def __init__(self, sex: str, mass: float, model: biorbd.Model):
        """
        Parameters
        ----------
        sex
            The sex of the subject ('male', 'female')
        mass
            The mass of the subject
        model
            The biorbd model. This is to compute lengths
        """
        self.sex = sex
        self.mass = mass
        self.model = model

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

    @property
    def segment_names(self) -> tuple[str, ...]:
        return tuple(self.table["male"].keys())

    def segment_mass(self, segment: str) -> float:
        return self.table[self.sex][segment].mass * self.mass

    def segment_length(self, segment: str) -> float:
        table = self.table[self.sex][segment]

        # Find the position of the markers when the model is in resting position
        marker_positions = np.array([marker.to_array() for marker in self.model.markers(self.q_zero)]).transpose()

        # Find the index of the markers required to compute the length of the segment
        idx_proximal = self.marker_names.index(table.marker_names[0])
        idx_distal = self.marker_names.index(table.marker_names[1])

        # Compute the Euclidian distance between the two positions
        return np.linalg.norm(marker_positions[:3, idx_distal] - marker_positions[:3, idx_proximal])

    def segment_center_of_mass(self, segment: str, inverse_proximal: bool = False) -> tuple[float, float, float]:
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

    def segment_moment_of_inertia(self, segment: str) -> tuple[float, float, float]:
        mass = self.segment_mass(segment)
        length = self.segment_length(segment)
        radii = self.table[self.sex][segment].radii

        return mass * (length * radii[0]) ** 2, mass * (length * radii[1]) ** 2, mass * (length * radii[2]) ** 2
