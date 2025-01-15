from typing import Callable
from enum import Enum

from .protocols import Data
from .muscle_real import MuscleReal, MuscleType, MuscleStateType


class Muscle:
    def __init__(
        self,
        name: str,
        muscle_type: MuscleType,
        state_type: MuscleStateType,
        muscle_group: str,
        origin_position_function: Callable,
        insertion_position_function: Callable,
        optimal_length_function: Callable | float,
        maximal_force_function: Callable | float,
        tendon_slack_length_function: Callable | float,
        pennation_angle_function: Callable | float,
        maximal_excitation: float = 1,
    ):
        """
        Parameters
        ----------
        name
            The name of the muscle
        muscle_type
            The type of the muscle
        """
        self.name = name
        self.muscle_type = muscle_type
        self.state_type = state_type
        self.muscle_group = muscle_group
        self.origin_position_function = origin_position_function
        self.insertion_position_function = insertion_position_function
        self.optimal_length_function = optimal_length_function
        self.maximal_force_function = maximal_force_function
        self.tendon_slack_length_function = tendon_slack_length_function
        self.pennation_angle_function = pennation_angle_function
        self.maximal_excitation = maximal_excitation

    def to_muscle(self, model, data: Data) -> MuscleReal:
        return MuscleReal.from_data(
            data,
            model,
            self.name,
            self.muscle_type,
            self.state_type,
            self.muscle_group,
            self.origin_position_function,
            self.insertion_position_function,
            self.optimal_length_function,
            self.maximal_force_function,
            self.tendon_slack_length_function,
            self.pennation_angle_function,
            self.maximal_excitation,
        )
