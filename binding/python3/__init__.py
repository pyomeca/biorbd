from enum import Enum, auto
from typing import Any

import numpy as np

from . import biorbd  # This is created while installing using CMake
from .biorbd import *
from ._version import __version__
from .surface_max_torque_actuator import *
from .rigid_body import *
from .utils import *


# Additional imports for casadi backend
if biorbd.currentLinearAlgebraBackend() == biorbd.CASADI:
    from casadi import MX, SX, DM, Function, horzcat


# If we are using the Eigen backend, returns are np.arrays, if we are using CasADi backend, returns are MX
# So declare an "Array" typedef that will be np.array or MX depending on the backend
if biorbd.currentLinearAlgebraBackend() == biorbd.CASADI:
    type BiorbdArray = MX
else:
    type BiorbdArray = np.ndarray


# Declare a method that converts output to BiorbdArray
def _to_biorbd_array_output(x: Any) -> BiorbdArray:
    if biorbd.currentLinearAlgebraBackend() == biorbd.EIGEN3:
        return x.to_array()
    elif biorbd.currentLinearAlgebraBackend() == biorbd.CASADI:
        return x.to_mx()
    else:
        raise NotImplementedError("Unknown backend")


def _to_biorbd_array_input(x: BiorbdArray) -> Any:
    if biorbd.currentLinearAlgebraBackend() == biorbd.EIGEN3:
        if isinstance(x, np.ndarray):
            return x
        return np.array(x)

    elif biorbd.currentLinearAlgebraBackend() == biorbd.CASADI:
        if isinstance(x, (MX, SX, DM)):
            return x
        return DM(x)
    else:
        raise NotImplementedError("Unknown backend")


class ReferenceFrame(Enum):
    LOCAL = auto()
    GLOBAL = auto()


class ExternalForceSet:
    class _Type(Enum):
        FORCE = auto()
        SPATIAL_VECTOR = auto()

    def __init__(self, external_force_set: biorbd.ExternalForceSet):
        self._external_force_set = external_force_set

    @classmethod
    def from_model(cls, model: "Biorbd"):
        return cls(model.internal.externalForceSet())

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
        reference_frame: ReferenceFrame = ReferenceFrame.GLOBAL,
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
            The force is expressed in the reference frame specified by the reference_frame parameter.
        point_of_application: BiorbdArray, optional
            The point of application of the force in the local frame of the segment.
            This must be a 3D vector ([px, py, pz]).
            This parameter is required when adding a force in the local reference frame (see reference_frame parameter).
            It is optional when adding a force in the global reference frame:
                - If a 3D force is added, this parameter is required.
                - If a 6D spatial vector is added, this parameter is ignored and can be set to None.
            Default is None.
        reference_frame: ReferenceFrame, optional
            The reference frame in which the force is expressed.
        """

        # Prepare the force vector properly
        force = _to_biorbd_array_input(force)
        if point_of_application is not None:
            point_of_application = _to_biorbd_array_input(point_of_application)

        vector_type = None
        if force.shape[0] == 3:
            vector_type = ExternalForceSet._Type.FORCE
        elif force.shape[0] == 6:
            vector_type = ExternalForceSet._Type.SPATIAL_VECTOR
            force = biorbd.SpatialVector(force)
        else:
            raise ValueError("The input vector must be of size 3 (force) or 6 (spatial vector)")

        # Add the actual force vector to the correct force set
        if vector_type == ExternalForceSet._Type.FORCE and reference_frame == ReferenceFrame.GLOBAL:
            if point_of_application is None:
                raise ValueError(
                    "The point of application must be provided when adding a force in the global reference frame"
                )
            self._external_force_set.addTranslationalForce(force, segment_name, point_of_application)

        elif vector_type == ExternalForceSet._Type.FORCE and reference_frame == ReferenceFrame.LOCAL:
            raise ValueError(
                "Adding a force in local reference frame is not implemented (and probably not what you want). "
                "You probably want to add a spatial vector in the local reference frame instead or a force in the global reference frame."
            )

        elif vector_type == ExternalForceSet._Type.SPATIAL_VECTOR and reference_frame == ReferenceFrame.GLOBAL:
            # It is correct to pass None as point of application, it is assumed to be applied at the origin of the global reference frame
            self._external_force_set.add(segment_name, force, point_of_application)

        elif vector_type == ExternalForceSet._Type.SPATIAL_VECTOR and reference_frame == ReferenceFrame.LOCAL:
            if point_of_application is None:
                raise ValueError(
                    "The point of application must be provided when adding a spatial vector in the local reference frame"
                )
            self._external_force_set.addInSegmentReferenceFrame(segment_name, force, point_of_application)

        else:
            raise NotImplementedError("The combination of vector type and reference frame is not implemented yet")


class Segment:
    def __init__(self, segment: biorbd.Segment):
        self._segment = segment

    @property
    def name(self) -> str:
        """
        Get the name of the segment.

        Returns
        -------
        The name of the segment.
        """
        return self._segment.name().to_string()

    @property
    def mass(self) -> float:
        """
        Get the mass of the segment.

        Returns
        -------
        The mass of the segment.
        """
        return self._characteristics.mass()

    @mass.setter
    def mass(self, value: float):
        """
        Set the mass of the segment.

        Parameters
        ----------
        value: float
            The mass of the segment.
        """
        self._characteristics.setMass(value)

    @property
    def internal(self) -> biorbd.Segment:
        """
        Get the internal segment of the Segment instance.

        Returns
        -------
        The internal segment. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._segment

    @property
    def _characteristics(self) -> biorbd.SegmentCharacteristics:
        return self._segment.characteristics()


class Biorbd:
    def __init__(self, path):
        self._model = biorbd.Model(path)
        self._external_force_set = None

    @property
    def internal(self) -> biorbd.Model:
        """
        Get the internal model of the Biorbd instance.

        Returns
        -------
        The internal model. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model

    @property
    def segments(self) -> list[Segment]:
        """
        Get the segments in the model.

        Returns
        -------
        A list of segments
        """
        return [Segment(seg) for seg in self._model.segments()]

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
            self._external_force_set = ExternalForceSet.from_model(self)
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

        input_parameters = [_to_biorbd_array_input(q), _to_biorbd_array_input(qdot), _to_biorbd_array_input(tau)]

        if not ignore_contacts and return_contact_forces:
            cs = self._model.getConstraints()
            input_parameters.append(cs)

        if not ignore_external_forces and self._external_force_set is not None:
            input_parameters.append(self.external_force_set._external_force_set)

        if ignore_contacts:
            return _to_biorbd_array_output(self._model.ForwardDynamics(*input_parameters))
        else:
            qddot = _to_biorbd_array_output(self._model.ForwardDynamicsConstraintsDirect(*input_parameters))

            if return_contact_forces:
                contact_forces = _to_biorbd_array_output(cs.getForce())
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
        input_parameters = [_to_biorbd_array_input(q), _to_biorbd_array_input(qdot), _to_biorbd_array_input(qddot)]

        if not ignore_external_forces and self._external_force_set is not None:
            input_parameters.append(self.external_force_set._external_force_set)

        return _to_biorbd_array_output(self._model.InverseDynamics(*input_parameters))


if biorbd.currentLinearAlgebraBackend() == 1:

    def to_casadi_func(name, func, *all_param, expand=True):
        cx_param = []
        for p in all_param:
            if isinstance(p, (MX, SX)):
                cx_param.append(p)

        if isinstance(func, (MX, SX, Function)):
            func_evaluated = func
        else:
            func_evaluated = func(*all_param)
            if isinstance(func_evaluated, (list, tuple)):
                func_evaluated = horzcat(*[val if isinstance(val, MX) else val.to_mx() for val in func_evaluated])
            elif not isinstance(func_evaluated, MX):
                func_evaluated = func_evaluated.to_mx()
        func = Function(name, cx_param, [func_evaluated])
        return func.expand() if expand else func
