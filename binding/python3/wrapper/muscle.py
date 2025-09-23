from collections import UserList
from typing import TYPE_CHECKING, Iterator

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
from .misc import BiorbdArray, BiorbdScalar, to_biorbd_array_input, to_biorbd_array_output
from ..biorbd import (
    Muscle as MuscleBiorbd,
    State,
    GeneralizedCoordinates,
    GeneralizedVelocity,
    currentLinearAlgebraBackend,
    CASADI,
    EIGEN3,
)

if currentLinearAlgebraBackend() == CASADI:
    from casadi import vertcat
elif currentLinearAlgebraBackend() == EIGEN3:
    from numpy import concatenate as vertcat


class Muscle:
    def __init__(self, model: "Biorbd", index: int):
        self._model = model
        self._index = index

    @property
    def name(self) -> str:
        """
        Get the name of the muscle.

        Returns
        -------
        The name of the muscle.
        """
        return self.internal.name().to_string()

    @property
    def optimal_length(self) -> BiorbdScalar:
        """
        Get the optimal length of the muscle.

        Returns
        -------
        The optimal length of the muscle.
        """
        return self.internal.characteristics().optimalLength()

    @optimal_length.setter
    def optimal_length(self, value: BiorbdScalar):
        """
        Set the optimal length of the muscle.

        Parameters
        ----------
        value: The new optimal length of the muscle.
        """
        self.internal.characteristics().setOptimalLength(value)

    @property
    def maximal_isometric_force(self) -> BiorbdScalar:
        """
        Get the maximal isometric force of the muscle.

        Returns
        -------
        The maximal isometric force of the muscle.
        """
        return self.internal.characteristics().forceIsoMax()

    @maximal_isometric_force.setter
    def maximal_isometric_force(self, value: BiorbdScalar):
        """
        Set the maximal isometric force of the muscle.

        Parameters
        ----------
        value: The new maximal isometric force of the muscle.
        """
        self.internal.characteristics().setForceIsoMax(value)

    @property
    def pcsa(self) -> BiorbdScalar:
        """
        Get the physiological cross-sectional area of the muscle.

        Returns
        -------
        The physiological cross-sectional area of the muscle.
        """
        return self.internal.characteristics().PCSA()

    @pcsa.setter
    def pcsa(self, value: BiorbdScalar):
        """
        Set the physiological cross-sectional area of the muscle.

        Parameters
        ----------
        value: The new physiological cross-sectional area of the muscle.
        """
        self.internal.characteristics().setPCSA(value)

    @property
    def tendon_slack_length(self) -> BiorbdScalar:
        """
        Get the tendon slack length of the muscle.

        Returns
        -------
        The tendon slack length of the muscle.
        """
        return self.internal.characteristics().tendonSlackLength()

    @tendon_slack_length.setter
    def tendon_slack_length(self, value: BiorbdScalar):
        """
        Set the tendon slack length of the muscle.

        Parameters
        ----------
        value: The new tendon slack length of the muscle.
        """
        self.internal.characteristics().setTendonSlackLength(value)

    @property
    def pennation_angle(self) -> BiorbdScalar:
        """
        Get the pennation angle of the muscle.

        Returns
        -------
        The pennation angle of the muscle.
        """
        return self.internal.characteristics().pennationAngle()

    @pennation_angle.setter
    def pennation_angle(self, value: BiorbdScalar):
        """
        Set the pennation angle of the muscle.

        Parameters
        ----------
        value: The new pennation angle of the muscle.
        """
        self.internal.characteristics().setPennationAngle(value)

    @property
    def maximal_contraction_velocity(self) -> BiorbdScalar:
        """
        Get the maximal contraction velocity of the muscle.

        Returns
        -------
        The maximal contraction velocity of the muscle.
        """
        return self.internal.characteristics().maxShorteningSpeed()

    @maximal_contraction_velocity.setter
    def maximal_contraction_velocity(self, value: BiorbdScalar):
        """
        Set the maximal contraction velocity of the muscle.

        Parameters
        ----------
        value: The new maximal contraction velocity of the muscle.
        """
        self.internal.characteristics().setMaxShorteningSpeed(value)

    @property
    def length(self) -> BiorbdScalar:
        """
        Get the current pre-updated muscle length of the muscle (update_geometry must therefore be called prior to this method).

        Returns
        -------
        The current length of the muscle.
        """
        dummy_q = GeneralizedCoordinates(self._model.nb_q)
        update_kinematics = False
        return self.internal.length(self._model.internal, dummy_q, update_kinematics)

    @property
    def muscle_tendon_length(self) -> BiorbdScalar:
        """
        Get the current pre-updated muscle-tendon length of the muscle (update_geometry must therefore be called prior to this method).

        Returns
        -------
        The current muscle-tendon length of the muscle.
        """
        dummy_q = GeneralizedCoordinates(self._model.nb_q)
        update_kinematics = False
        return self.internal.musculoTendonLength(self._model.internal, dummy_q, update_kinematics)

    @property
    def velocity(self) -> BiorbdScalar:
        """
        Get the current pre-updated lengthening velocity of the muscle (update_geometry must therefore be called prior to this method).

        Returns
        -------
        The current lengthening velocity of the muscle.
        """
        dummy_q = GeneralizedCoordinates(self._model.nb_q)
        dummy_qdot = GeneralizedVelocity(self._model.nb_qdot)
        update_kinematics = False
        return self.internal.velocity(self._model.internal, dummy_q, dummy_qdot, update_kinematics)

    @property
    def length_jacobian(self) -> BiorbdArray:
        """
        Get the current pre-updated jacobian of the muscle (update_geometry must therefore be called prior to this method).

        Returns
        -------
        The current jacobian of the muscle.
        """
        return to_biorbd_array_output(self.internal.position().jacobianLength())

    @property
    def activation(self) -> BiorbdScalar:
        """
        Get the current activation of the muscle.

        Returns
        -------
        The current activation of the muscle.
        """

        return self._state.activation()

    @activation.setter
    def activation(self, value: BiorbdScalar):
        """
        Set the current activation of the muscle.

        Parameters
        ----------
        value: The new activation of the muscle.
        """
        self._state.setActivation(value)

    @property
    def excitation(self) -> BiorbdScalar:
        """
        Get the current excitation of the muscle.

        Returns
        -------
        The current excitation of the muscle.
        """
        return self._state.excitation()

    @excitation.setter
    def excitation(self, value: BiorbdScalar):
        """
        Set the current excitation of the muscle.

        Parameters
        ----------
        value: The new excitation of the muscle.
        """
        self._state.setExcitation(value)

    def activation_dot(self, excitation: BiorbdScalar | None, activation: BiorbdScalar | None) -> BiorbdScalar:
        """
        Compute the time derivative of the muscle activation.

        Parameters
        ----------
        excitation: BiorbdScalar | None
            The current excitation of the muscle. If None is provided, the previously set excitation is used.
        activation: BiorbdScalar | None
            The current activation of the muscle. If None is provided, the previously set activation is used.

        Returns
        -------
        The time derivative of the muscle activation.
        """
        excitation_is_normalized = True  # excitation is between 0 and 1
        if excitation is not None:
            self.excitation = excitation
        if activation is not None:
            self.activation = activation

        return self.internal.activationDot(self._state, excitation_is_normalized)

    @property
    def force(self) -> BiorbdScalar:
        """
        Get the current force of the muscle (update_geometry must therefore be called prior to this method).

        Returns
        -------
        The current force of the muscle.
        """
        return self.internal.force(self._state)

    @property
    def _state(self) -> State:
        """Get the internal state of the muscle stored in the biorbd model."""
        return self._model.internal.stateSet()[self._index]

    @property
    def internal(self) -> MuscleBiorbd:
        """
        Get the internal muscle of the Muscle instance.

        Returns
        -------
        The internal muscle. It can be used to access any resources that are not yet wrapped in Python binder.
        """
        return self._model.internal.muscle(self._index)


class MusclesList(UserList):
    data: list[Muscle]

    def __init__(self, muscles: list[Muscle], model: "Biorbd"):
        super().__init__(muscles)
        self._model = model

    def __getitem__(self, item: str | int) -> Muscle:
        if isinstance(item, str):
            for muscle in self.data:
                if muscle.name == item:
                    return muscle
            raise KeyError(f"Muscle {item} not found")

        return self.data[item]

    def __iter__(self) -> Iterator[Muscle]:
        return super().__iter__()

    @property
    def activations(self) -> list[BiorbdArray]:
        """
        Get the current activations of all muscles.

        Returns
        -------
        The current activations of all muscles.
        """
        return [muscle.activation for muscle in self.data]

    @activations.setter
    def activations(self, values: BiorbdArray):
        """
        Set the current activations of all muscles.

        Parameters
        ----------
        values: The new activations of all muscles.
        """
        if len(values) != len(self.data):
            raise ValueError(
                f"Length of values ({len(values)}) must be equal to the number of muscles ({len(self.data)})"
            )
        for muscle, value in zip(self.data, values):
            muscle.activation = value

    @property
    def excitations(self) -> list[BiorbdArray]:
        """
        Get the current excitations of all muscles.

        Returns
        -------
        The current excitations of all muscles.
        """
        return [muscle.excitation for muscle in self.data]

    @excitations.setter
    def excitations(self, values: BiorbdArray):
        """
        Set the current excitations of all muscles.

        Parameters
        ----------
        values: The new excitations of all muscles.
        """
        if len(values) != len(self.data):
            raise ValueError(
                f"Length of values ({len(values)}) must be equal to the number of muscles ({len(self.data)})"
            )
        for muscle, value in zip(self.data, values):
            muscle.excitation = value

    def activations_dot(self, excitations: BiorbdArray, activations: BiorbdArray) -> BiorbdArray:
        """
        Compute the time derivative of the muscle activations.

        Parameters
        ----------
        excitations: BiorbdArray
            The current excitations of the muscles.
        activations: BiorbdArray
            The current activations of the muscles.

        Returns
        -------
        The time derivative of the muscle activations.
        """
        if len(excitations) != len(self.data):
            raise ValueError(
                f"Length of excitations ({len(excitations)}) must be equal to the number of muscles ({len(self.data)})"
            )
        if len(activations) != len(self.data):
            raise ValueError(
                f"Length of activations ({len(activations)}) must be equal to the number of muscles ({len(self.data)})"
            )

        return [
            muscle.activation_dot(excitation, activation)
            for muscle, excitation, activation in zip(self.data, excitations, activations)
        ]

    def update_geometry(self, q: BiorbdArray | None = None, qdot: BiorbdArray | None = None):
        """
        Force the update of the muscle geometry to a new pose and velocity.

        Parameters
        ----------
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities
        """
        if q is None and qdot is None:
            # Nothing to do
            return

        input_parameters = []
        if q is not None:
            input_parameters.append(to_biorbd_array_input(q))
        if qdot is not None:
            input_parameters.append(to_biorbd_array_input(qdot))
        input_parameters.append(True)  # Always update the muscle lengths

        self._model.internal.updateMuscles(*input_parameters)

    def forces(
        self, activations: BiorbdArray | None = None, q: BiorbdArray | None = None, qdot: BiorbdArray | None = None
    ) -> BiorbdArray:
        """
        Compute the muscle forces at a given pose, velocity and activation.

        Parameters
        ----------
        activations: BiorbdArray
            Muscle activations
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities

        Returns
        -------
        The muscle forces.
        """

        self.update_geometry(q, qdot)
        if activations is not None:
            self.activations = activations
        emg = self._model.internal.stateSet()
        return to_biorbd_array_output(self._model.internal.muscleForces(emg))

    def joint_torque(
        self, activations: BiorbdArray | None = None, q: BiorbdArray | None = None, qdot: BiorbdArray | None = None
    ) -> BiorbdArray:
        """
        Compute the joint torque generated by the muscles at a given pose and velocity. If no activations are provided
        the model is not updated at all and returns the previously computed joint torques. If activations are provided
        but no pose or velocity, the model is updated only with the activations. If activations, pose and velocity
        are provided, the model is fully updated (slowest)

        Parameters
        ----------
        activations: BiorbdArray
            Muscle activations
        q: BiorbdArray
            Generalized coordinates
        qdot: BiorbdArray
            Generalized velocities

        Returns
        -------
        The joint torque generated by the muscles.
        """
        self.update_geometry(q, qdot)
        if activations is not None:
            self.activations = activations

        state_set = self._model.internal.stateSet()
        return to_biorbd_array_output(self._model.internal.muscularJointTorque(state_set))

    def length_jacobian(
        self,
        q: BiorbdArray | None = None,
    ) -> BiorbdArray:
        """
        Get the current length jacobian of all muscles at q (if provided, otherwise at the current pose).

        Returns
        -------
        The current length jacobian of all muscles.
        """
        self.update_geometry(q)
        return vertcat([muscle.length_jacobian for muscle in self.data])
