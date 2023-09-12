import numpy as np
from . import biorbd  # This is created while installing using CMake


def to_spatial_vector(f_ext: np.ndarray) -> biorbd.VecBiorbdSpatialVector:
    """
    Converts a 6 x n_external_force np.array into biorbd spatial vector


    Parameters
    ----------
    f_ext: np.ndarray
        The array to convert

    Returns
    -------
    The conveted array
    """

    vector = biorbd.VecBiorbdSpatialVector()
    for idx in range(f_ext.shape[1]):
        vector.append(biorbd.SpatialVector(f_ext[:, idx]))
    return vector


def transport_spatial_force(
        spatial_force:  biorbd.SpatialVector,
        current_application_point: np.ndarray,
        new_application_point: np.ndarray,
) -> biorbd.SpatialVector:
    """
    Transports a spatial force from one point to another
    """

    forces = spatial_force.to_array()[3:]
    torques = spatial_force.to_array()[:3]
    # Torque transport, Bour's formula
    torques += np.cross(current_application_point - new_application_point,
            forces)

    return biorbd.SpatialVector(np.concatenate((torques, forces)))


class ExternalForce:
    """
    This class allow to manage external forces efficiently
    """
    def __init__(self,
                 force_in_global: np.ndarray,
                 torque_in_global: np.ndarray,
                 application_point_in_global: np.ndarray,
                ):
        """
        Parameters
        ----------
        force_in_global: np.ndarray
            The force in global reference frame
        torque_in_global: np.ndarray
            The torque in global reference frame
        application_point_in_global: np.ndarray
            The application point in global reference frame
        """
        self._force_in_global = force_in_global
        self._torque_in_global = torque_in_global
        self._application_point_in_global = application_point_in_global

    @property
    def spatial_vector_at_origin(self):
        """
        Returns
        -------
        The spatial vector at the origin. This is the required format for the external forces in biorbd
        and in the formalism of Featherstone (Rigid Body Dynamics Algorithm)
        """

        spatial_vector = transport_spatial_force(
            biorbd.SpatialVector(np.concatenate((self._torque_in_global, self._force_in_global))),
            self._application_point_in_global,
            np.zeros(3),
        )

        return spatial_vector

    @property
    def spatial_vector(self):
        """
        Returns
        -------
        The spatial vector.
        """

        return biorbd.SpatialVector(np.concatenate((self._torque_in_global, self._force_in_global)))

    @classmethod
    def from_local_coordinate_system(cls,
                                     force_in_local: np.ndarray,
                                     torque_in_local: np.ndarray,
                                     application_point_in_local: np.ndarray,
                                     rototranslation_of_local_system_in_global: np.ndarray,
                                     ):
        """
        This function creates an external force from a local coordinate system

        Parameters
        ----------
        force_in_local: np.ndarray
            The force in local reference frame
        torque_in_local: np.ndarray
            The torque in local reference frame
        application_point_in_local: np.ndarray
            The application point in local reference frame
        rototranslation_of_local_system_in_global: np.ndarray
            The rototranslation of the local system in global reference frame [4 x 4], aka homogeneous matrix

        Returns
        -------
        The external force object
        """

        # Convert the force and torque to global reference frame
        force_in_global = rototranslation_of_local_system_in_global[:3, :3] @ force_in_local
        torque_in_global = rototranslation_of_local_system_in_global[:3, :3] @ torque_in_local

        # Convert the application point to global reference frame
        application_point_in_global = (rototranslation_of_local_system_in_global[:3, :3] @ application_point_in_local
                                       + rototranslation_of_local_system_in_global[:3, 3])

        return cls(force_in_global, torque_in_global, application_point_in_global)
