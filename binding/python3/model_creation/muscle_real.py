from typing import Callable

import numpy as np
from enum import Enum

from .protocols import Data


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
        type: MuscleType,
        state_type: MuscleStateType,
        muscle_group: str,
        origin_position: tuple[int | float, int | float, int | float] | np.ndarray,
        insertion_position: tuple[int | float, int | float, int | float] | np.ndarray,
        optimal_length: float,
        maximal_force: float,
        tendon_slack_length: float,
        pennation_angle: float,
        maximal_excitation: float,
    ):
        """
        Parameters
        ----------
        name
            The name of the new contact
        type
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
        """
        self.name = name
        self.type = type
        self.state_type = state_type
        self.muscle_group = muscle_group
        self.origin_position = origin_position if isinstance(origin_position, np.ndarray) else np.array(origin_position)
        self.insertion_position = insertion_position if isinstance(insertion_position, np.ndarray) else np.array(insertion_position)
        self.optimal_length = optimal_length
        self.maximal_force = maximal_force
        self.tendon_slack_length = tendon_slack_length
        self.pennation_angle = pennation_angle
        self.maximal_excitation = maximal_excitation


    @staticmethod
    def from_data(
        data: Data,
        name: str,
        type: MuscleType,
        state_type: MuscleStateType,
        muscle_group: str,
        origin_position_function: Callable | np.ndarray[float],
        insertion_position_function: Callable | np.ndarray[float],
        optimal_length_function: Callable | float,
        maximal_force_function: Callable | float,
        tendon_slack_length_function: Callable | float,
        pennation_angle_function: Callable | float,
        maximal_excitation: float
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
        type
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
            The function (f(m) -> float, where m is a dict of markers) that defines the optimal length of the muscle
        maximal_force_function
            The function (f(m) -> float, where m is a dict of markers) that defines the maximal force of the muscle
        tendon_slack_length_function
            The function (f(m) -> float, where m is a dict of markers) that defines the tendon slack length of the muscle
        pennation_angle_function
            The function (f(m) -> float, where m is a dict of markers) that defines the pennation angle of the muscle
        maximal_excitation
            The maximal excitation of the muscle (usually 1.0, since it is normalized)
        """

        origin_position: np.ndarray = origin_position_function(data.values)
        if not isinstance(origin_position, np.ndarray):
            raise RuntimeError(f"The origin_position_function {origin_position_function} must return a np.ndarray of dimension 3xT (XYZ x time)")
        if len(origin_position.shape) == 1:
            origin_position = origin_position[:, np.newaxis]
        if len(origin_position.shape) != 2 or origin_position.shape[0] != 3:
            raise RuntimeError(f"The origin_position_function {origin_position_function} must return a np.ndarray of dimension 3xT (XYZ x time)")

        insertion_position: np.ndarray = insertion_position_function(data.values)
        if not isinstance(insertion_position, np.ndarray):
            raise RuntimeError(f"The insertion_position_function {insertion_position_function} must return a np.ndarray of dimension 3xT (XYZ x time)")
        if len(insertion_position.shape) == 1:
            insertion_position = insertion_position[:, np.newaxis]
        if len(origin_position) != 2 or insertion_position.shape[0] != 3:
            raise RuntimeError(f"The insertion_position_function {insertion_position_function} must return a np.ndarray of dimension 3xT (XYZ x time)")

        optimal_length: float = optimal_length_function(data.values)
        if not isinstance(optimal_length, float):
            raise RuntimeError(f"The optimal_length_function {optimal_length_function} must return a float")

        maximal_force: float = maximal_force_function(data.values)
        if not isinstance(maximal_force, float):
            raise RuntimeError(f"The maximal_force_function {maximal_force_function} must return a float")

        tendon_slack_length: float = tendon_slack_length_function(data.values)
        if not isinstance(tendon_slack_length, float):
            raise RuntimeError(f"The tendon_slack_length_function {tendon_slack_length_function} must return a float")

        pennation_angle: float = pennation_angle_function(data.values)
        if not isinstance(pennation_angle, float):
            raise RuntimeError(f"The pennation_angle_function {pennation_angle_function} must return a float")

        return MuscleReal(name,
                            type,
                            state_type,
                            muscle_group,
                            origin_position,
                            insertion_position,
                            optimal_length,
                            maximal_force,
                            tendon_slack_length,
                            pennation_angle,
                            maximal_excitation)

    def __str__(self):
        # Define the print function, so it automatically formats things in the file properly
        out_string = f"muscle {self.name}\n"
        out_string += f"\ttype: {self.type}\n"
        out_string += f"\tstatetype {self.state_type}\n"
        out_string += f"\tmusclegroup {self.muscle_group}\n"
        out_string += f"\toriginposition {self.origin_position}\n"
        out_string += f"\tinsertionposition {self.insertion_position}\n"
        out_string += f"\toptimallength {self.optimal_length}\n"
        out_string += f"\tmaximalforce {self.maximal_force}\n"
        out_string += f"\ttendonslacklength {self.tendon_slack_length}\n"
        out_string += f"\tpennationangle {self.penation_angle}\n"
        out_string += "endmuscle\n"

        # Define each muscle's via points
        for i_viapoint, via_point in enumerate(self.via_points):
            out_string += f"viapoint {via_point.name}\n"
            out_string += f"\tparent {via_point.parent_name}\n"
            out_string += f"\tmuscle {via_point.muscle_name}\n"
            out_string += f"\tmusclegroup {via_point.muscle_group}\n"
            out_string += f"\tposition {via_point.position}\n"
            out_string += "endviapoint\n"

        return out_string
