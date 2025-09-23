from enum import Enum, auto

from .misc import BiorbdArray, to_biorbd_array_input
from ..biorbd import ExternalForceSet as BiorbdExternalForceSet, SpatialVector as BiorbdSpatialVector


class ExternalForceSet:
    class Frame(Enum):
        LOCAL = auto()
        WORLD = auto()

    class _Type(Enum):
        FORCE = auto()
        SPATIAL_VECTOR = auto()

    def __init__(self, external_force_set: BiorbdExternalForceSet):
        self._external_force_set = external_force_set

    def reset(self) -> None:
        """
        Remove all the external forces from the set. This must be called before adding new external forces.
        """
        self._external_force_set.setZero()

    def add(
        self,
        segment_name: str,
        force: BiorbdArray,
        point_of_application: BiorbdArray | None = None,
        frame_of_reference: Frame = Frame.WORLD,
    ) -> None:
        """
        Add an external force to the external force set. From now on, this force will be apply when calling other methods
        such as forward_dynamics or inverse_dynamics. If called multiple times, the forces will be accumulated. To remove
        all the forces, call the reset() method and then add the new forces. It is not possible to remove a single force from the set.

        Parameters
        ----------
        segment_name: str
            The name of the segment to which the force is applied. This is not verified to be a valid segment name.
            If it is not a valid name, you may encounter an error "RuntimeError: Asked for a wrong segment (out of range)"
            when calling other methods such as forward_dynamics or inverse_dynamics.
        force: BiorbdArray
            The force vector to be applied to the segment.
            This must be a  3D vector (for translational forces, that is [fx, fy, fz]) or a
                            6D vector (for spatial forces, that is [mx, my, mz, fx, fy, fz]).
            The force is expressed in the frame specified by the frame_of_reference parameter.
        point_of_application: BiorbdArray, optional
            The point of application of the force in the local frame of the segment.
            This must be a 3D vector ([px, py, pz]).
            This parameter is required when adding a force in the local frame (see frame_of_reference parameter).
            It is optional when adding a force in the world frame:
                - If a 3D force is added, this parameter is required.
                - If a 6D spatial vector is added, this parameter is ignored and can be set to None.
            Default is None.
        frame_of_reference: Frame, optional
            The frame in which the force is expressed.
        """

        # Prepare the force vector properly
        force = to_biorbd_array_input(force)
        if point_of_application is not None:
            point_of_application = to_biorbd_array_input(point_of_application)

        vector_type = None
        if force.shape[0] == 3:
            vector_type = ExternalForceSet._Type.FORCE
        elif force.shape[0] == 6:
            vector_type = ExternalForceSet._Type.SPATIAL_VECTOR
            force = BiorbdSpatialVector(force)
        else:
            raise ValueError("The input vector must be of size 3 (force) or 6 (spatial vector)")

        # Add the actual force vector to the correct force set
        if vector_type == ExternalForceSet._Type.FORCE and frame_of_reference == ExternalForceSet.Frame.WORLD:
            if point_of_application is None:
                raise ValueError("The point of application must be provided when adding a force in the world frame")
            self._external_force_set.addTranslationalForce(force, segment_name, point_of_application)

        elif vector_type == ExternalForceSet._Type.FORCE and frame_of_reference == ExternalForceSet.Frame.LOCAL:
            raise ValueError(
                "Adding a force in local frame is not implemented (and probably not what you want). "
                "You probably want to add a spatial vector in the local frame instead or a force in the world frame."
            )

        elif (
            vector_type == ExternalForceSet._Type.SPATIAL_VECTOR and frame_of_reference == ExternalForceSet.Frame.WORLD
        ):
            # It is correct to pass None as point of application, it is assumed to be applied at the origin of the world frame
            self._external_force_set.add(segment_name, force, point_of_application)

        elif (
            vector_type == ExternalForceSet._Type.SPATIAL_VECTOR and frame_of_reference == ExternalForceSet.Frame.LOCAL
        ):
            if point_of_application is None:
                raise ValueError(
                    "The point of application must be provided when adding a spatial vector in the local frame"
                )
            self._external_force_set.addInSegmentReferenceFrame(segment_name, force, point_of_application)

        else:
            raise NotImplementedError("The combination of vector type and frame is not implemented yet")
