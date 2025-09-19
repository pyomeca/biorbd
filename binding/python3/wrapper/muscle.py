from collections import UserList
from typing import TYPE_CHECKING, Iterator

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
from .misc import BiorbdArray, to_biorbd_array_input, to_biorbd_array_output
from ..biorbd import Muscle as MuscleBiorbd, State, GeneralizedCoordinates, GeneralizedVelocity


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
    def activation(self) -> float:
        """
        Get the current activation of the muscle.

        Returns
        -------
        The current activation of the muscle.
        """

        return self._state.activation()

    @activation.setter
    def activation(self, value: float):
        """
        Set the current activation of the muscle.

        Parameters
        ----------
        value: The new activation of the muscle.
        """
        self._state.setActivation(value)

    @property
    def excitation(self) -> float:
        """
        Get the current excitation of the muscle.

        Returns
        -------
        The current excitation of the muscle.
        """
        return self._state.excitation()

    @excitation.setter
    def excitation(self, value: float):
        """
        Set the current excitation of the muscle.

        Parameters
        ----------
        value: The new excitation of the muscle.
        """
        self._state.setExcitation(value)

    def activation_dot(self, excitation: float | None, activation: float | None) -> float:
        """
        Compute the time derivative of the muscle activation.

        Parameters
        ----------
        excitation: float | None
            The current excitation of the muscle. If None is provided, the previously set excitation is used.
        activation: float | None
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

        update_level = 0
        if activations is not None:
            self.activations = activations
            update_level = 1

        state_set = self._model.internal.stateSet()
        q = GeneralizedCoordinates(self._model.nb_q) if q is None else to_biorbd_array_input(q)
        qdot = GeneralizedVelocity(self._model.nb_qdot) if qdot is None else to_biorbd_array_input(qdot)

        return to_biorbd_array_output(
            self._model.internal.muscleForces(
                state_set, GeneralizedCoordinates(q), GeneralizedVelocity(qdot), update_level
            )
        )

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
