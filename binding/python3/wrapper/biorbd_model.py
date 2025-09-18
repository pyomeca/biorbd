from collections import UserList
from typing import Iterator

from .external_force_set import ExternalForceSet
from .markers import Marker, GlobalMarker
from .misc import BiorbdArray, to_biorbd_array_input, to_biorbd_array_output
from .segment import Segment
from ..biorbd import Model


class Biorbd:
    def __init__(self, path):
        self._model = Model(path)
        self._external_force_set = None

    @property
    def internal(self) -> Model:
        """
        Get the internal model of the Biorbd instance.

        Returns
        -------
        The internal model. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model

    @property
    def segments(self) -> "SegmentsList":
        """
        Get the segments in the model.

        Returns
        -------
        A list of segments
        """
        return SegmentsList([Segment(seg) for seg in self._model.segments()])

    @property
    def markers(self) -> "MarkersList":
        """
        Get the markers attached to the model.

        Returns
        -------
        A list of markers
        """
        return MarkersList([Marker(marker) for marker in self._model.markers()], model=self)

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


class SegmentsList(UserList):
    data: list[Segment]

    def __getitem__(self, item: str | int) -> Segment:
        if isinstance(item, str):
            for seg in self.data:
                if seg.name == item:
                    return seg
            raise KeyError(f"Segment {item} not found")

        return self.data[item]


class MarkersList(UserList):
    data: list[Marker]

    def __init__(self, markers: list[Marker], model: Biorbd):
        super().__init__(markers)
        self._model = model

    def __getitem__(self, item: str | int) -> Marker:
        if isinstance(item, str):
            for marker in self.data:
                if marker.name == item:
                    return marker
            raise KeyError(f"Marker {item} not found")

        return self.data[item]

    def __iter__(self) -> Iterator[Marker]:
        return super().__iter__()

    def __call__(self, q: BiorbdArray, update_kinematics: bool = True) -> "MarkersList":
        """
        Perform of forward kinematics to get the position of the markers at a given pose.
        """
        q = to_biorbd_array_input(q)
        return MarkersList(
            [GlobalMarker(marker) for marker in self._model.internal.markers(q, update_kinematics)], model=None
        )
