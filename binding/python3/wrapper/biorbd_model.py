from pathlib import Path

from .external_force_set import ExternalForceSet
from .marker import Marker, MarkersList
from .misc import BiorbdArray, BiorbdScalar, to_biorbd_array_input, to_biorbd_array_output
from .muscle import Muscle, MusclesList
from .segment import Segment, SegmentsList
from .segment_frame import SegmentFrame, SegmentFramesList
from ..biorbd import Model, GeneralizedCoordinates, currentLinearAlgebraBackend, CASADI


class Biorbd:
    def __init__(self, path):
        self._model = Model(path)
        self._external_force_set = None

    @property
    def name(self) -> str:
        """
        Get the name of the model.

        Returns
        -------
        The name of the model
        """
        return self.internal.path().filename().to_string()

    @property
    def path(self) -> Path:
        """
        Get the path of the model.

        Returns
        -------
        The path of the model
        """
        return self._model.path().relativePath().to_string()

    @property
    def gravity(self) -> BiorbdArray:
        """
        Get the gravity vector of the model.

        Returns
        -------
        The gravity vector of the model
        """
        return to_biorbd_array_output(self._model.getGravity())

    @gravity.setter
    def gravity(self, new_gravity: BiorbdArray):
        """
        Set the gravity vector of the model.

        Parameters
        ----------
        new_gravity: BiorbdArray
            The new gravity vector of the model
        """
        self._model.setGravity(to_biorbd_array_input(new_gravity))

    @property
    def mass(self) -> BiorbdScalar:
        """
        Get the mass of the model.

        Returns
        -------
        The mass of the model
        """
        return to_biorbd_array_output(self._model.mass())

    def mass_matrix(self, q: BiorbdArray | None = None, inverse: bool = False) -> BiorbdArray:
        """
        Get the mass matrix of the model at a given pose (or the pose already set in the model if q is None).

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        inverse: bool
            If true, the inverse of the mass matrix is returned. Default is False.

        Returns
        -------
        The mass matrix of the model
        """
        if q is None and currentLinearAlgebraBackend() == CASADI:
            raise RuntimeError(
                "The 'mass_matrix' method without setting q cannot be called when using the CasADi backend"
            )

        updated_model = self.update_kinematics(q)
        dummy_q = GeneralizedCoordinates(self.nb_q) if q is None else to_biorbd_array_input(q)
        update_kinematics = False
        if inverse:
            return to_biorbd_array_output(updated_model.massMatrixInverse(dummy_q, update_kinematics))
        else:
            return to_biorbd_array_output(updated_model.massMatrix(dummy_q, update_kinematics))

    def center_of_mass(self, q: BiorbdArray | None = None) -> BiorbdArray:
        """
        Get the center of mass of the model at a given pose (or the pose already set in the model if q is None).

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates

        Returns
        -------
        The center of mass of the model
        """
        if q is None and currentLinearAlgebraBackend() == CASADI:
            raise RuntimeError(
                "The 'center_of_mass' method without setting q cannot be called when using the CasADi backend"
            )

        updated_model = self.update_kinematics(q)
        dummy_q = GeneralizedCoordinates(self.nb_q) if q is None else to_biorbd_array_input(q)
        update_kinematics = False
        return to_biorbd_array_output(updated_model.CoM(dummy_q, update_kinematics))

    @property
    def segments(self) -> SegmentsList:
        """
        Get the segments in the model.

        Returns
        -------
        A list of segments
        """
        return SegmentsList([Segment(self, index) for index in range(self._model.nbSegment())])

    @property
    def markers(self) -> MarkersList:
        """
        Get the markers attached to the model.

        Returns
        -------
        A list of markers
        """
        return MarkersList([Marker(self, index) for index in range(self._model.nbMarkers())], model=self)

    @property
    def segment_frames(self) -> SegmentFramesList:
        """
        Get the segment frames in the model.

        Returns
        -------
        A list of segment frames
        """
        return SegmentFramesList([SegmentFrame(self, index) for index in range(self._model.nbSegment())], model=self)

    @property
    def dof_names(self) -> list[str]:
        """
        Get the names of the degrees of freedom in the model.

        Returns
        -------
        A list of degree of freedom names
        """
        return [dof.to_string() for dof in self._model.nameDof()]

    @property
    def nb_q(self) -> int:
        """
        Get the number of generalized coordinates in the model.

        Returns
        -------
        The number of generalized coordinates
        """
        return self._model.nbQ()

    @property
    def nb_qdot(self) -> int:
        """
        Get the number of generalized velocities in the model.

        Returns
        -------
        The number of generalized velocities
        """
        return self._model.nbQdot()

    @property
    def nb_qddot(self) -> int:
        """
        Get the number of generalized accelerations in the model.

        Returns
        -------
        The number of generalized accelerations
        """
        return self._model.nbQddot()

    @property
    def nb_tau(self) -> int:
        """
        Get the number of generalized torques in the model.

        Returns
        -------
        The number of generalized torques
        """
        return self._model.nbGeneralizedTorque()

    @property
    def external_force_set(self) -> ExternalForceSet:
        """
        Get the external force set of the model.

        Returns
        -------
        The external force set of the model.
        """
        if self._external_force_set is None:
            self._external_force_set = ExternalForceSet(external_force_set=self._model.externalForceSet())
        return self._external_force_set

    @property
    def muscles(self) -> "MusclesList":
        """
        Get the muscles in the model.

        Returns
        -------
        A list of muscles
        """
        return MusclesList([Muscle(self, index) for index in range(self._model.nbMuscles())], model=self)

    @property
    def internal(self) -> Model:
        """
        Get the internal model of the Biorbd instance.

        Returns
        -------
        The internal model. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model

    def update_kinematics(
        self, q: BiorbdArray | None = None, qdot: BiorbdArray | None = None, qddot: BiorbdArray | None = None
    ) -> Model:
        """
        Force the update the model to a new pose, velocity and acceleration.

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities
        qddot: BiorbdArray
            Generalized accelerations

        Returns
        -------
        The q, qdot and qddot used to update the model (None if not used)
        """
        q = None if q is None else to_biorbd_array_input(q)
        qdot = None if qdot is None else to_biorbd_array_input(qdot)
        qddot = None if qddot is None else to_biorbd_array_input(qddot)

        return self._model.UpdateKinematicsCustom(q, qdot, qddot)

    def forward_dynamics(
        self,
        q: BiorbdArray,
        qdot: BiorbdArray,
        tau: BiorbdArray,
        ignore_external_forces: bool = False,
        ignore_contacts: bool = False,
        return_contact_forces: bool = False,
    ) -> BiorbdArray | tuple[BiorbdArray, BiorbdArray]:
        """
        Perform forward dynamics on the model.

        Note: If [external_force_set] has been modified (by adding external forces), these forces will NOT be taken into account.

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities
        tau: BiorbdArray
            Generalized forces
        ignore_contacts: bool, optional
            If true, contact forces will be ignored. Default is False.
        ignore_external_forces: bool, optional
            If true, external forces will be ignored. Default is False.
        return_contact_forces: bool, optional
            If true, the contact forces will be returned as an extra parameter. Default is False.
            If [ignore_contacts] is True, this parameter is ignored.

        Returns
        -------
        Generalized accelerations
        """
        # Set the ignore_contacts to True if there is no contact in the model
        if self._model.nbContacts() == 0:
            ignore_contacts = True

        input_parameters = [to_biorbd_array_input(q), to_biorbd_array_input(qdot), to_biorbd_array_input(tau)]

        if not ignore_contacts and return_contact_forces:
            cs = self._model.getConstraints()
            input_parameters.append(cs)

        if not ignore_external_forces and self._external_force_set is not None:
            input_parameters.append(self.external_force_set._external_force_set)

        if ignore_contacts:
            return to_biorbd_array_output(self._model.ForwardDynamics(*input_parameters))
        else:
            qddot = to_biorbd_array_output(self._model.ForwardDynamicsConstraintsDirect(*input_parameters))

            if return_contact_forces:
                contact_forces = to_biorbd_array_output(cs.getForce())
                return qddot, contact_forces
            else:
                return qddot

    def inverse_dynamics(
        self, q: BiorbdArray, qdot: BiorbdArray, qddot: BiorbdArray, ignore_external_forces: bool = False
    ) -> BiorbdArray:
        """
        Perform inverse dynamics on the model.

        Note: If [external_force_set] has been modified (by adding external forces), these forces will NOT be taken into account.

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities
        qddot: BiorbdArray
            Generalized accelerations
        ignore_external_forces: bool, optional
            If true, external forces will be ignored. Default is False.

        Returns
        -------
        Generalized forces
        """
        input_parameters = [to_biorbd_array_input(q), to_biorbd_array_input(qdot), to_biorbd_array_input(qddot)]

        if not ignore_external_forces and self._external_force_set is not None:
            input_parameters.append(self.external_force_set._external_force_set)

        return to_biorbd_array_output(self._model.InverseDynamics(*input_parameters))
