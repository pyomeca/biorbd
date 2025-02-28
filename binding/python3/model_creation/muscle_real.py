from typing import Callable

import numpy as np
from enum import Enum

from .protocols import Data
from .via_point_real import ViaPointReal


class MuscleType(Enum):
    HILLTHELEN = "hillthelen"
    # TODO: @pariterre to be completed


class MuscleStateType(Enum):
    DEGROOTE = "DeGroote"
    # TODO: @pariterre to be completed


class MuscleReal:
    def __init__(
        self,
        name: str,
        muscle_type: MuscleType,
        state_type: MuscleStateType,
        muscle_group: str,
        origin_position: tuple[int | float, int | float, int | float] | np.ndarray,
        insertion_position: tuple[int | float, int | float, int | float] | np.ndarray,
        optimal_length: float,
        maximal_force: float,
        tendon_slack_length: float,
        pennation_angle: float,
        maximal_excitation: float,
        via_points: list[ViaPointReal] = None,
    ):
        """
        Parameters
        ----------
        name
            The name of the new contact
        muscle_type
            The type of the muscle
        state_type
            The state type of the muscle
        muscle_group
            The muscle group the muscle belongs to
        origin_position
            The origin position of the muscle in the local reference frame of the origin segment @pariterre: please confirm
        insertion_position
            The insertion position of the muscle the local reference frame of the insertion segment @pariterre: please confirm
        optimal_length
            The optimal length of the muscle
        maximal_force
            The maximal force of the muscle can reach
        tendon_slack_length
            The length of the tendon at rest
        pennation_angle
            The pennation angle of the muscle
        maximal_excitation
            The maximal excitation of the muscle (usually 1.0, since it is normalized)
        via_points
            The via points of the muscle
        """
        self.name = name
        self.muscle_type = muscle_type
        self.state_type = state_type
        self.muscle_group = muscle_group
        self.origin_position = origin_position if isinstance(origin_position, np.ndarray) else np.array(origin_position)
        self.insertion_position = (
            insertion_position if isinstance(insertion_position, np.ndarray) else np.array(insertion_position)
        )
        self.optimal_length = optimal_length
        self.maximal_force = maximal_force
        self.tendon_slack_length = tendon_slack_length
        self.pennation_angle = pennation_angle
        self.maximal_excitation = maximal_excitation
        self.via_points = via_points

    @staticmethod
    def from_data(
        data: Data,
        model,
        name: str,
        muscle_type: MuscleType,
        state_type: MuscleStateType,
        muscle_group: str,
        origin_position_function: Callable | np.ndarray[float],
        insertion_position_function: Callable | np.ndarray[float],
        optimal_length_function: Callable | float,
        maximal_force_function: Callable | float,
        tendon_slack_length_function: Callable | float,
        pennation_angle_function: Callable | float,
        maximal_excitation: float,
    ):
        """
        This is a constructor for the Muscle class. It evaluates the function that defines the muscle to get an
        actual position

        Parameters
        ----------
        data
            The data to pick the data from
        name
            The name of the muscle
        muscle_type
            The type of the muscle
        state_type
            The state type of the muscle
        muscle_group
            The muscle group the muscle belongs to
        origin_position_function
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the origin position of the muscle
        insertion_position_function
            The function (f(m) -> np.ndarray, where m is a dict of markers) that defines the insertion position of the muscle
        optimal_length_function
            The function (f(model, m) -> float, where m is a dict of markers) that defines the optimal length of the muscle
        maximal_force_function
            The function (f(m) -> float, where m is a dict of markers) that defines the maximal force of the muscle
        tendon_slack_length_function
            The function (f(model, m) -> float, where m is a dict of markers) that defines the tendon slack length of the muscle
        pennation_angle_function
            The function (f(model, m) -> float, where m is a dict of markers) that defines the pennation angle of the muscle
        maximal_excitation
            The maximal excitation of the muscle (usually 1.0, since it is normalized)
        """
        origin_position: np.ndarray = origin_position_function(data.values)
        if not isinstance(origin_position, np.ndarray):
            raise RuntimeError(
                f"The origin_position_function {origin_position_function} must return a vector of dimension 3 (XYZ)"
            )
        if origin_position.shape == (3, 1):
            origin_position = origin_position.reshape((3,))
        elif origin_position.shape != (3,):
            raise RuntimeError(
                f"The origin_position_function {origin_position_function} must return a vector of dimension 3 (XYZ)"
            )

        insertion_position: np.ndarray = insertion_position_function(data.values)
        if not isinstance(insertion_position, np.ndarray):
            raise RuntimeError(
                f"The insertion_position_function {insertion_position_function} must return a vector of dimension 3 (XYZ)"
            )
        if insertion_position.shape == (3, 1):
            insertion_position = insertion_position.reshape((3,))
        elif insertion_position.shape != (3,):
            raise RuntimeError(
                f"The insertion_position_function {insertion_position_function} must return a vector of dimension 3 (XYZ)"
            )

        optimal_length: float = optimal_length_function(model, data.values)
        if not isinstance(optimal_length, float):
            raise RuntimeError(f"The optimal_length_function {optimal_length_function} must return a float")

        maximal_force: float = maximal_force_function(data.values)
        if not isinstance(maximal_force, float):
            raise RuntimeError(f"The maximal_force_function {maximal_force_function} must return a float")

        tendon_slack_length: float = tendon_slack_length_function(model, data.values)
        if not isinstance(tendon_slack_length, float):
            raise RuntimeError(f"The tendon_slack_length_function {tendon_slack_length_function} must return a float")

        pennation_angle: float = pennation_angle_function(model, data.values)
        if not isinstance(pennation_angle, float):
            raise RuntimeError(f"The pennation_angle_function {pennation_angle_function} must return a float")

        return MuscleReal(
            name,
            muscle_type,
            state_type,
            muscle_group,
            origin_position,
            insertion_position,
            optimal_length,
            maximal_force,
            tendon_slack_length,
            pennation_angle,
            maximal_excitation,
        )

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"muscle\t{self.name}\n"
        out_string += f"\ttype\t{self.muscle_type.value}\n"
        out_string += f"\tstatetype\t{self.state_type.value}\n"
        out_string += f"\tmusclegroup\t{self.muscle_group}\n"
        out_string += f"\toriginposition\t{np.round(self.origin_position[0], 4)}\t{np.round(self.origin_position[1], 4)}\t{np.round(self.origin_position[2], 4)}\n"
        out_string += f"\tinsertionposition\t{np.round(self.insertion_position[0], 4)}\t{np.round(self.insertion_position[1], 4)}\t{np.round(self.insertion_position[2], 4)}\n"
        out_string += f"\toptimallength\t{self.optimal_length:0.4f}\n"
        out_string += f"\tmaximalforce\t{self.maximal_force:0.4f}\n"
        out_string += f"\ttendonslacklength\t{self.tendon_slack_length:0.4f}\n"
        out_string += f"\tpennationangle\t{self.pennation_angle:0.4f}\n"
        out_string += "endmuscle\n"
        return out_string
